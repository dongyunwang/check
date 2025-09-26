/*******************************************************
 * Gemini-Mod-Log (Cumulative)
 *
 * --- VERSION 1 ---
 * Date: 2025-09-19
 * Changes:
 *  - Implemented a robust startup handshake protocol.
 *  - Added discoverAndVerifySlaves() function to ensure all slaves are
 *    connected before starting the main sequence.
 *  - Modified setup() to loop until all slaves are found.
 *  - Added Serial output for debugging the discovery process.
 ******************************************************

 *******************************************************
 * ESP32 Master - 展览主机（按“第一个方案”最小修复版）
 * 修复要点：
 * 1) 在 runCascadePhase() 与 runRandomPhase() 进入阶段时置 system_running=true，
 *    确保 onDataRecv() 中的级联触发 updateCascadeTrigger() 能被调用。
 * 2) 进入级联阶段时将 device_last_pos 初始化为参考设备当前遥测值，
 *    避免刚进入阶段的初始抖动被累计触发。
 *
 * 说明：
 * - 未改动通信结构体、命令编码与其余逻辑。
 * - CASCADE_MAX_DISTANCE 仍未参与运算（沿用你原设计）。
 ******************************************************


#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <EEPROM.h>
#include <math.h>
#include <string.h>

// ========== 可修改参数（仅此处保留中文注释） ==========
#define TARGET_CYCLES_CASCADE  3    // 级联模式次数（设为0则跳过级联）
#define TARGET_CYCLES_RANDOM   3    // 随机模式次数（设为0则跳过随机）

#define EEPROM_SIZE        8        // EEPROM大小（如未使用可保持不变）
#define WIFI_CHANNEL       1        // Wi-Fi信道（需与从机一致）
#define NUM_SLAVES         6        // 从机数量

#define CASCADE_TRIGGER_DISTANCE  5.0f   // 级联触发间隔（参考设备每累计位移多少cm触发下一台）
#define CASCADE_MAX_DISTANCE      19.0f  // 级联最大位移（可与从机最大行程对齐）（当前未使用）
#define CASCADE_REF_DEVICE        1      // 级联参考设备ID（1~NUM_SLAVES）

#define EXHIBIT_TELE_INTERVAL_MS  200    // 遥测上报请求间隔（毫秒）

static const float    READY_POS_CM         = 1.0f;  // 回零后就绪位置（cm）
static const float    READY_TOL_CM         = 0.6f;  // 就绪位置容差（cm）
static const uint32_t READY_TIMEOUT_MS     = 8000;  // 就绪等待超时（ms）
static const uint32_t PING_INTERVAL_MS     = 200;   // PING轮询间隔（ms）

// 设备启用（true=参与，false=跳过），按实际接线启用/禁用
static bool ENABLED[NUM_SLAVES] = { 
  true, true, true, true, true, true
};

// 从机MAC地址表（与实物对应，按需修改）
static uint8_t SLAVE_MACS[NUM_SLAVES][6] = {
  {0xA0,0xA3,0xB3,0x2B,0x78,0x74},
  {0xA0,0xA3,0xB3,0x28,0x12,0x18},
  {0x08,0xD1,0xF9,0xD1,0xA3,0x20},
  {0xFC,0xF5,0xC4,0x4B,0xB9,0x5C},
  {0xA0,0xA3,0xB3,0x2B,0x45,0x0C},
  {0xA0,0xA3,0xB3,0x29,0x94,0x94}
};
// ====================================================

struct __attribute__((packed)) ControlMsg {
  uint8_t  cmd;
  uint8_t  target_id;
  float    start_cm;
  float    end_cm;
  float    trigger_cm;
  float    max_cm;
  float    speed;
  float    accel;
  uint8_t  mode;
  uint32_t seq;
  uint16_t max_cycles;
  uint16_t tele_interval_ms;
};

struct __attribute__((packed)) TelemetryMsg {
  uint8_t  slave_id;
  uint8_t  status;
  float    pos_cm;
  uint32_t cycles;
  uint32_t seq_ack;
};

static const float DEFAULT_END_CM     = 19.5f;
static const float DEFAULT_TRIGGER_CM = 5.0f;
static const float DEFAULT_SPEED      = 4000.0f;
static const float DEFAULT_ACCEL      = 800.0f;

static bool system_running = false;
static uint32_t g_seq = 1;

enum Phase { PHASE_CASCADE, PHASE_RANDOM };
static Phase current_phase = PHASE_CASCADE;

static bool   cascade_device_started[NUM_SLAVES] = {false};
static float  device_accumulated_distance = 0;
static float  device_last_pos = 0;

static bool     device_completed[NUM_SLAVES] = {false};
static uint32_t device_cycles[NUM_SLAVES]    = {0};

static float    last_pos_cm[NUM_SLAVES]  = {0};
static uint32_t last_seen_ms[NUM_SLAVES] = {0};
static uint32_t last_print_cycles[NUM_SLAVES] = {0};
static bool     slave_responded[NUM_SLAVES] = {false};
static uint8_t  last_status[NUM_SLAVES] = {0};

static void onDataRecv(const uint8_t* mac, const uint8_t* data, int len);
static void onDataSent(const uint8_t* mac, esp_now_send_status_t status);
static void runExhibitionSequence();
static void ensureHomeAll();
static void runCascadePhase();
static void runRandomPhase();
static void stopAll();
static bool waitForAtPos(uint8_t slave_id, float target_cm, float tol_cm, uint32_t timeout_ms);
static bool initESPNow();
static bool addPeers();
static void sendCommand(uint8_t cmd, uint8_t slave_id, float end_cm=DEFAULT_END_CM, 
                       float trigger_cm=DEFAULT_TRIGGER_CM, uint8_t mode=0, 
                       float speed=DEFAULT_SPEED, float accel=DEFAULT_ACCEL, 
                       uint16_t max_cycles=100);

static int getEnabledCount(){
  int c=0; 
  for(int i=0;i<NUM_SLAVES;i++) if (ENABLED[i]) c++; 
  return c;
}
static void resetCompletionStatus(){
  for(int i=0;i<NUM_SLAVES;i++){
    device_completed[i]=false; 
    device_cycles[i]=0; 
    last_print_cycles[i]=0; 
    slave_responded[i]=false; 
  }
}
static bool checkAllDevicesComplete(){
  for(int i=0;i<NUM_SLAVES;i++)
    if (ENABLED[i] && !device_completed[i]) return false; 
  return true;
}

static bool initESPNow(){
  WiFi.mode(WIFI_STA); 
  WiFi.disconnect(); 
  esp_wifi_set_ps(WIFI_PS_NONE); 
  delay(60);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false); 
  delay(60);
  if (esp_now_init() != ESP_OK) { 
    return false; 
  }
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);
  return true;
}

static bool addPeers(){
  for(int i=0;i<NUM_SLAVES;i++){
    if (!ENABLED[i]) continue;
    esp_now_peer_info_t p={}; 
    memcpy(p.peer_addr,SLAVE_MACS[i],6);
    p.channel=WIFI_CHANNEL; 
    p.encrypt=false; 
    p.ifidx=WIFI_IF_STA;
    esp_now_add_peer(&p);
  }
  return true;
}

static void sendCommand(uint8_t cmd, uint8_t slave_id, float end_cm, float trigger_cm,
                       uint8_t mode, float speed, float accel, uint16_t max_cycles){
  if (slave_id<1 || slave_id>NUM_SLAVES || !ENABLED[slave_id-1]) return;
  ControlMsg m={};
  m.cmd=cmd; 
  m.target_id=slave_id;
  m.end_cm=end_cm; 
  m.trigger_cm=trigger_cm; 
  m.max_cm=end_cm;
  m.speed=speed; 
  m.accel=accel; 
  m.mode=mode; 
  m.seq=g_seq++; 
  m.max_cycles=max_cycles;
  m.tele_interval_ms = EXHIBIT_TELE_INTERVAL_MS;
  esp_now_send(SLAVE_MACS[slave_id-1], (uint8_t*)&m, sizeof(m));
}

static bool waitForAtPos(uint8_t sid, float target_cm, float tol_cm, uint32_t timeout_ms){
  if (!ENABLED[sid-1]) return true;
  uint32_t t0=millis();
  while(millis()-t0<timeout_ms){
    if (fabsf(last_pos_cm[sid-1]-target_cm)<=tol_cm) return true;
    sendCommand(4, sid, 0, 0, 0, 0, 0, 0);
    delay(100);
  }
  return false;
}

static void ensureHomeAll(){
  for(uint8_t i=1;i<=NUM_SLAVES;i++) 
    if (ENABLED[i-1]) { sendCommand(2,i,0,0,0,0,0,0); delay(20); } 
  delay(500);
  for(uint8_t i=1;i<=NUM_SLAVES;i++) 
    if (ENABLED[i-1]) { sendCommand(6,i,0,0,0,0,0,0); delay(20); }

  bool all_ready = false;
  uint32_t last_status_check = 0;
  uint32_t start_time = millis();
  while(!all_ready) {
    all_ready = true;
    if (millis() - last_status_check > PING_INTERVAL_MS) {
      last_status_check = millis();
      for(uint8_t i=1; i<=NUM_SLAVES; i++){
        if (!ENABLED[i-1]) continue;
        sendCommand(4, i, 0, 0, 0, 0, 0, 0);
        delay(50);
        float pos = last_pos_cm[i-1];
        bool at_ready = (fabsf(pos - READY_POS_CM) <= READY_TOL_CM);
        if (!at_ready) all_ready = false;
      }
    }
    if (millis() - start_time > READY_TIMEOUT_MS) break;
    delay(10);
  }
  for(uint8_t i=1; i<=NUM_SLAVES; i++){
    if (!ENABLED[i-1]) continue;
    sendCommand(4, i, 0, 0, 0, 0, 0, 0);
    delay(30);
  }
  delay(200);
}

static void updateCascadeTrigger(float ref_pos){
  float delta=fabsf(ref_pos-device_last_pos);
  device_accumulated_distance += delta;
  device_last_pos = ref_pos;
  if (device_accumulated_distance >= CASCADE_TRIGGER_DISTANCE){
    for(int i=1; i<NUM_SLAVES; i++){
      if (ENABLED[i] && !cascade_device_started[i]){
        // 触发下一台（模式=0 级联）
        sendCommand(1, i+1, DEFAULT_END_CM, 0, 0, DEFAULT_SPEED, DEFAULT_ACCEL, TARGET_CYCLES_CASCADE);
        cascade_device_started[i]=true;
        device_accumulated_distance -= CASCADE_TRIGGER_DISTANCE;
        break;
      }
    }
  }
}

static void onDataRecv(const uint8_t* mac, const uint8_t* data, int len){
  if (len!=(int)sizeof(TelemetryMsg)) return;
  TelemetryMsg t;
  memcpy(&t,data,sizeof(t));
  if (t.slave_id<1 || t.slave_id>NUM_SLAVES || !ENABLED[t.slave_id-1]) return;
  last_pos_cm[t.slave_id-1] = t.pos_cm;
  last_seen_ms[t.slave_id-1] = millis();
  device_cycles[t.slave_id-1] = t.cycles;
  slave_responded[t.slave_id-1] = true;
  last_status[t.slave_id-1] = t.status;

  // 保持原有触发条件：需要 system_running=true + 处于级联阶段 + 参考设备数据
  if (system_running && current_phase==PHASE_CASCADE && t.slave_id==CASCADE_REF_DEVICE){
    updateCascadeTrigger(t.pos_cm);
  }

  if (t.status==10){ 
    device_completed[t.slave_id-1]=true;
  }
}

static void onDataSent(const uint8_t* mac, esp_now_send_status_t status){}

static void stopAll(){
  for(uint8_t i=1;i<=NUM_SLAVES;i++) 
    if (ENABLED[i-1]) { sendCommand(2,i,0,0,0,0,0,0); delay(20); }
  system_running=false; // 阶段结束时关掉，进入下一阶段前再打开（见修复）
}

static void runCascadePhase(){
  if (TARGET_CYCLES_CASCADE == 0) return;
  current_phase = PHASE_CASCADE;
  system_running = true;   // ★ 修复：确保 onDataRecv 中的级联触发生效
  resetCompletionStatus();
  memset(cascade_device_started, 0, sizeof(cascade_device_started));
  device_accumulated_distance = 0;

  // ★ 修复：以参考设备“当前遥测位置”为基准，避免初始抖动累计
  device_last_pos = last_pos_cm[CASCADE_REF_DEVICE-1];

  // 启动参考设备（模式=0 级联）
  sendCommand(1, CASCADE_REF_DEVICE, DEFAULT_END_CM, DEFAULT_TRIGGER_CM, 0, 
              DEFAULT_SPEED, DEFAULT_ACCEL, TARGET_CYCLES_CASCADE);
  cascade_device_started[CASCADE_REF_DEVICE-1] = true;

  while (!checkAllDevicesComplete()) {
    delay(100);
  }
  delay(200);
}

static void runRandomPhase(){
  if (TARGET_CYCLES_RANDOM == 0) return;
  current_phase = PHASE_RANDOM;
  system_running = true;   // ★ 修复：保持运行标志，稳定接收遥测
  resetCompletionStatus();
  for(uint8_t i=1; i<=NUM_SLAVES; i++){
    if (!ENABLED[i-1]) continue;
    // 模式=1 随机
    sendCommand(1, i, DEFAULT_END_CM, 0, 1, DEFAULT_SPEED, DEFAULT_ACCEL, TARGET_CYCLES_RANDOM);
    delay(1000);
  }
  while (!checkAllDevicesComplete()) {
    delay(100);
  }
  delay(200);
}

static void runExhibitionSequence(){
  ensureHomeAll();
  system_running = true; // 第一次进入时打开；阶段内会再次显式打开（见修复）
  while(true){
    if (TARGET_CYCLES_CASCADE > 0) {
      runCascadePhase();
      stopAll();
      ensureHomeAll();
    }
    if (TARGET_CYCLES_RANDOM > 0) {
      runRandomPhase();
      stopAll();
      ensureHomeAll();
    }
    if (TARGET_CYCLES_CASCADE == 0 && TARGET_CYCLES_RANDOM == 0) {
      break;
    }
  }
}

static bool discoverAndVerifySlaves() {
      Serial.printf("Discovering slaves... looking for %d devices.\n", getEnabledCount());
      resetCompletionStatus(); // This also resets the slave_responded array

      // Send out a burst of pings to everyone to ensure delivery
      for (int i = 0; i < 3; i++) {
        for(uint8_t j=1; j<=NUM_SLAVES; j++) {
          if (ENABLED[j-1]) {
            sendCommand(4, j, 0, 0, 0, 0, 0, 0); // PING
            delay(20);
          }
        }
      }

      Serial.println("Waiting for responses...");
      delay(2000); // Wait 2 seconds for all slaves to reply

      int found_count = 0;
      for(int i=0; i<NUM_SLAVES; i++) {
        if(ENABLED[i] && slave_responded[i]) {
          found_count++;
          Serial.printf("  - Slave %d responded.\n", i+1);
        } else if (ENABLED[i]) {
          Serial.printf("  - Slave %d DID NOT respond.\n", i+1);
        }
      }

      if (found_count >= getEnabledCount()) {
        Serial.println("Success! All enabled slaves are online.");
        return true;
      } else {
        Serial.printf("Failure: Only found %d out of %d slaves.\n", found_count, getEnabledCount());
        return false;
      }
    }

void setup(){
  Serial.begin(115200);
  Serial.println("\nMaster Controller starting up...");

  if (!initESPNow()) {
    Serial.println("ESP-NOW initialization failed. Restarting...");
  }
  addPeers();
  
  while (!discoverAndVerifySlaves()) {
     Serial.println("Discovery failed. Retrying in 3 seconds...");
     delay(3000);
  }

  Serial.println("All slaves found. Starting exhibition sequence.");
  runExhibitionSequence();
}

void loop(){
  delay(10);
}
