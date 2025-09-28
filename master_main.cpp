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
 * ESP32 Master - Stage3 协同主机实现
 * 核心要点：
 * 1) 以 SlaveState 聚合每台从机的在线、位置、循环数与命令 ACK，
 *    便于对 Stage3 级联/随机流程进行集中调度与监控。
 * 2) 所有关键指令携带序列号并等待从机在遥测中返回 ACK，
 *    失败时输出 Serial 警告以保证链路确认。
 * 3) 级联触发依据参考设备的累计位移逐台点火，可容纳禁用设备
 *    并在遥测缺失时安全回退。
 *******************************************************


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
static const uint8_t CASCADE_ORDER[NUM_SLAVES] = {
  1, 2, 3, 4, 5, 6
};

#define EXHIBIT_TELE_INTERVAL_MS  200    // 遥测上报请求间隔（毫秒）

static const float    READY_POS_CM         = 1.0f;  // 回零后就绪位置（cm）
static const float    READY_TOL_CM         = 0.6f;  // 就绪位置容差（cm）
static const uint32_t READY_TIMEOUT_MS     = 8000;  // 就绪等待超时（ms）
static const uint32_t PING_INTERVAL_MS     = 200;   // PING轮询间隔（ms）

// 设备启用（true=参与，false=跳过），按实际接线启用/禁用
static const bool DEFAULT_ENABLED[NUM_SLAVES] = {
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

enum Command : uint8_t {
  CMD_START = 1,
  CMD_STOP  = 2,
  CMD_PING  = 4,
  CMD_TEST  = 5,
  CMD_HOME  = 6
};

enum TelemetryStatus : uint8_t {
  TELE_IDLE       = 0,
  TELE_TRIGGERED  = 4,
  TELE_LINK_OK    = 5,
  TELE_HIT_LIMIT  = 8,
  TELE_TEST_DONE  = 9,
  TELE_CYCLE_DONE = 10
};

struct SlaveState {
  bool     enabled;
  bool     online;
  bool     responded;
  bool     completed;
  bool     cascade_launched;
  float    last_pos_cm;
  float    accum_distance_cm;
  uint32_t cycles;
  uint32_t last_seen_ms;
  uint32_t last_seq_ack;
  uint8_t  last_status;
};

static SlaveState slaves[NUM_SLAVES];
static int cascade_next_idx = 0;

static void onDataRecv(const uint8_t* mac, const uint8_t* data, int len);
static void onDataSent(const uint8_t* mac, esp_now_send_status_t status);
static void initSlaveStates();
static bool isSlaveEnabled(uint8_t slave_id);
static void runExhibitionSequence();
static void ensureHomeAll();
static void runCascadePhase();
static void runRandomPhase();
static void stopAll();
static bool waitForAtPos(uint8_t slave_id, float target_cm, float tol_cm, uint32_t timeout_ms);
static bool initESPNow();
static bool addPeers();
static uint32_t sendCommand(Command cmd, uint8_t slave_id, float end_cm=DEFAULT_END_CM,
                           float trigger_cm=DEFAULT_TRIGGER_CM, uint8_t mode=0,
                           float speed=DEFAULT_SPEED, float accel=DEFAULT_ACCEL,
                           uint16_t max_cycles=100);
static bool waitForAck(uint8_t slave_id, uint32_t seq, uint32_t timeout_ms=500);

static void initSlaveStates(){
  for(int i=0;i<NUM_SLAVES;i++){
    slaves[i].enabled            = DEFAULT_ENABLED[i];
    slaves[i].online             = false;
    slaves[i].responded          = false;
    slaves[i].completed          = false;
    slaves[i].cascade_launched   = false;
    slaves[i].last_pos_cm        = 0.0f;
    slaves[i].accum_distance_cm  = 0.0f;
    slaves[i].cycles             = 0;
    slaves[i].last_seen_ms       = 0;
    slaves[i].last_seq_ack       = 0;
    slaves[i].last_status        = 0;
  }
}

static bool isSlaveEnabled(uint8_t slave_id){
  if (slave_id<1 || slave_id>NUM_SLAVES) return false;
  return slaves[slave_id-1].enabled;
}

static int getEnabledCount(){
  int c=0;
  for(int i=0;i<NUM_SLAVES;i++) if (slaves[i].enabled) c++;
  return c;
}
static void resetCompletionStatus(){
  for(int i=0;i<NUM_SLAVES;i++){
    slaves[i].completed=false;
    slaves[i].responded=false;
    slaves[i].cycles=0;
    slaves[i].last_status=0;
    slaves[i].last_seq_ack=0;
    slaves[i].accum_distance_cm=0.0f;
  }
}
static bool checkAllDevicesComplete(){
  for(int i=0;i<NUM_SLAVES;i++)
    if (slaves[i].enabled && !slaves[i].completed) return false;
  return true;
}

static int findNextCascadeIndex(int start_idx){
  if (start_idx < 0) start_idx = 0;
  for (int offset = 0; offset < NUM_SLAVES; ++offset){
    int idx = (start_idx + offset) % NUM_SLAVES;
    uint8_t candidate_id = CASCADE_ORDER[idx];
    if (candidate_id < 1 || candidate_id > NUM_SLAVES) continue;
    if (candidate_id == CASCADE_REF_DEVICE) continue;
    if (!isSlaveEnabled(candidate_id)) continue;
    if (slaves[candidate_id-1].cascade_launched) continue;
    return idx;
  }
  return -1;
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
    if (!slaves[i].enabled) continue;
    esp_now_peer_info_t p={};
    memcpy(p.peer_addr,SLAVE_MACS[i],6);
    p.channel=WIFI_CHANNEL;
    p.encrypt=false;
    p.ifidx=WIFI_IF_STA;
    esp_now_add_peer(&p);
  }
  return true;
}

static uint32_t sendCommand(Command cmd, uint8_t slave_id, float end_cm, float trigger_cm,
                       uint8_t mode, float speed, float accel, uint16_t max_cycles){
  if (!isSlaveEnabled(slave_id)) return 0;
  ControlMsg m={};
  m.cmd=static_cast<uint8_t>(cmd);
  m.target_id=slave_id;
  m.end_cm=end_cm;
  m.trigger_cm=trigger_cm;
  m.max_cm=end_cm;
  m.speed=speed;
  m.accel=accel;
  m.mode=mode;
  uint32_t seq = g_seq++;
  m.seq=seq;
  m.max_cycles=max_cycles;
  m.tele_interval_ms = EXHIBIT_TELE_INTERVAL_MS;
  esp_now_send(SLAVE_MACS[slave_id-1], (uint8_t*)&m, sizeof(m));
  slaves[slave_id-1].last_seq_ack = 0;
  return seq;
}

static bool waitForAck(uint8_t slave_id, uint32_t seq, uint32_t timeout_ms){
  if (!isSlaveEnabled(slave_id) || seq==0) return true;
  uint32_t start = millis();
  while (millis()-start < timeout_ms){
    if (slaves[slave_id-1].last_seq_ack == seq) return true;
    delay(10);
  }
  return false;
}

static bool waitForAtPos(uint8_t sid, float target_cm, float tol_cm, uint32_t timeout_ms){
  if (!isSlaveEnabled(sid)) return true;
  uint32_t t0=millis();
  while(millis()-t0<timeout_ms){
    if (fabsf(slaves[sid-1].last_pos_cm-target_cm)<=tol_cm) return true;
    sendCommand(CMD_PING, sid, 0, 0, 0, 0, 0, 0);
    delay(100);
  }
  return false;
}

static void ensureHomeAll(){
  for(uint8_t i=1;i<=NUM_SLAVES;i++)
    if (isSlaveEnabled(i)) {
      uint32_t seq = sendCommand(CMD_STOP,i,0,0,0,0,0,0);
      if (!waitForAck(i, seq, 400)) {
        Serial.printf("[WARN] Stop command ack timeout for slave %u during homing\n", i);
      }
      delay(20);
    }
  delay(500);
  for(uint8_t i=1;i<=NUM_SLAVES;i++)
    if (isSlaveEnabled(i)) {
      uint32_t seq = sendCommand(CMD_HOME,i,0,0,0,0,0,0);
      if (!waitForAck(i, seq, 600)) {
        Serial.printf("[WARN] Home command ack timeout for slave %u\n", i);
      }
      delay(20);
    }

  bool all_ready = false;
  uint32_t last_status_check = 0;
  uint32_t start_time = millis();
  while(!all_ready) {
    all_ready = true;
    if (millis() - last_status_check > PING_INTERVAL_MS) {
      last_status_check = millis();
      for(uint8_t i=1; i<=NUM_SLAVES; i++){
        if (!isSlaveEnabled(i)) continue;
        sendCommand(CMD_PING, i, 0, 0, 0, 0, 0, 0);
        delay(50);
        float pos = slaves[i-1].last_pos_cm;
        bool at_ready = (fabsf(pos - READY_POS_CM) <= READY_TOL_CM);
        if (!at_ready) all_ready = false;
      }
    }
    if (millis() - start_time > READY_TIMEOUT_MS) break;
    delay(10);
  }
  for(uint8_t i=1; i<=NUM_SLAVES; i++){
    if (!isSlaveEnabled(i)) continue;
    sendCommand(CMD_PING, i, 0, 0, 0, 0, 0, 0);
    delay(30);
  }
  delay(200);
}

static void onDataRecv(const uint8_t* mac, const uint8_t* data, int len){
  if (len!=(int)sizeof(TelemetryMsg)) return;
  TelemetryMsg t;
  memcpy(&t,data,sizeof(t));
  if (!isSlaveEnabled(t.slave_id)) return;
  SlaveState &s = slaves[t.slave_id-1];
  float prev_pos = s.last_pos_cm;
  s.last_pos_cm = t.pos_cm;
  s.last_seen_ms = millis();
  s.cycles = t.cycles;
  s.responded = true;
  s.online = true;
  s.last_status = t.status;
  if (t.seq_ack != 0) s.last_seq_ack = t.seq_ack;

  // 保持原有触发条件：需要 system_running=true + 处于级联阶段 + 参考设备数据
  if (system_running && current_phase==PHASE_CASCADE && t.slave_id==CASCADE_REF_DEVICE){
    float delta = fabsf(t.pos_cm - prev_pos);
    if (delta > 0.0f){
      s.accum_distance_cm += delta;
      while (s.accum_distance_cm >= CASCADE_TRIGGER_DISTANCE){
        int idx = findNextCascadeIndex(cascade_next_idx);
        if (idx >= 0){
          uint8_t slave_id = CASCADE_ORDER[idx];
          sendCommand(CMD_START, slave_id, DEFAULT_END_CM, 0, 0,
                      DEFAULT_SPEED, DEFAULT_ACCEL, TARGET_CYCLES_CASCADE);
          slaves[slave_id-1].cascade_launched = true;
          cascade_next_idx = (idx + 1) % NUM_SLAVES;
          s.accum_distance_cm -= CASCADE_TRIGGER_DISTANCE;
        } else {
          s.accum_distance_cm = fmodf(s.accum_distance_cm, CASCADE_TRIGGER_DISTANCE);
          break;
        }
      }
    }
  }

  if (t.status==TELE_CYCLE_DONE){
    s.completed=true;
  } else if (t.status==TELE_HIT_LIMIT){
    Serial.printf("[WARN] Slave %u hit limit switch at %.2f cm\n", t.slave_id, t.pos_cm);
  }
}

static void onDataSent(const uint8_t* mac, esp_now_send_status_t status){}

static void stopAll(){
  for(uint8_t i=1;i<=NUM_SLAVES;i++)
    if (isSlaveEnabled(i)) {
      uint32_t seq = sendCommand(CMD_STOP,i,0,0,0,0,0,0);
      if (!waitForAck(i, seq, 400)) {
        Serial.printf("[WARN] Stop command ack timeout for slave %u\n", i);
      }
      delay(20);
    }
  system_running=false; // 阶段结束时关掉，进入下一阶段前再打开（见修复）
}

static void runCascadePhase(){
  if (TARGET_CYCLES_CASCADE == 0) return;
  current_phase = PHASE_CASCADE;
  system_running = true;   // ★ 修复：确保 onDataRecv 中的级联触发生效
  resetCompletionStatus();
  for(int i=0;i<NUM_SLAVES;i++){
    slaves[i].cascade_launched = false;
    slaves[i].accum_distance_cm = 0.0f;
  }

  int ref_idx = -1;
  for (int i=0; i<NUM_SLAVES; ++i){
    if (CASCADE_ORDER[i] == CASCADE_REF_DEVICE){
      ref_idx = i;
      break;
    }
  }
  if (ref_idx < 0){
    Serial.printf("[WARN] Reference device %u not found in cascade order, defaulting to ID order.\n", CASCADE_REF_DEVICE);
    cascade_next_idx = 0;
  } else {
    cascade_next_idx = (ref_idx + 1) % NUM_SLAVES;
  }

  // 启动参考设备（模式=0 级联）
  uint32_t seq = sendCommand(CMD_START, CASCADE_REF_DEVICE, DEFAULT_END_CM, DEFAULT_TRIGGER_CM, 0,
                             DEFAULT_SPEED, DEFAULT_ACCEL, TARGET_CYCLES_CASCADE);
  if (!waitForAck(CASCADE_REF_DEVICE, seq, 400)) {
    Serial.printf("[WARN] Cascade start ack timeout for ref slave %u\n", CASCADE_REF_DEVICE);
  }
  slaves[CASCADE_REF_DEVICE-1].cascade_launched = true;
  slaves[CASCADE_REF_DEVICE-1].accum_distance_cm = 0.0f;

  int next_idx = findNextCascadeIndex(cascade_next_idx);
  cascade_next_idx = (next_idx >= 0) ? (next_idx % NUM_SLAVES) : cascade_next_idx;

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
    if (!isSlaveEnabled(i)) continue;
    // 模式=1 随机
    uint32_t seq = sendCommand(CMD_START, i, DEFAULT_END_CM, 0, 1, DEFAULT_SPEED, DEFAULT_ACCEL, TARGET_CYCLES_RANDOM);
    if (!waitForAck(i, seq, 400)) {
      Serial.printf("[WARN] Random start ack timeout for slave %u\n", i);
    }
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
      for(int i=0;i<NUM_SLAVES;i++){
        slaves[i].responded = false;
        slaves[i].online = false;
      }

      // Send out a burst of pings to everyone to ensure delivery
      for (int i = 0; i < 3; i++) {
        for(uint8_t j=1; j<=NUM_SLAVES; j++) {
          if (isSlaveEnabled(j)) {
            sendCommand(CMD_PING, j, 0, 0, 0, 0, 0, 0);
            delay(20);
          }
        }
      }

      Serial.println("Waiting for responses...");
      delay(2000); // Wait 2 seconds for all slaves to reply

      int found_count = 0;
      for(int i=0; i<NUM_SLAVES; i++) {
        if(slaves[i].enabled && slaves[i].responded) {
          found_count++;
          Serial.printf("  - Slave %d responded.\n", i+1);
        } else if (slaves[i].enabled) {
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

  initSlaveStates();

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
