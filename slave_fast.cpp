/*******************************************************
 * ESP32 Slave - Plan B + CONFIG (CMD=7)
 * Date: 2025-09-28
 *
 * - 动态指派 ID（GROUP_SALT + ASSIGN），写入 EEPROM。
 * - 上电进入“发现窗口”（仅对 PING/ASSIGN 放宽，认主）；绑定后转严格模式。
 * - ★ 新增 CMD=7: CONFIG（一次性下发 home/work 速度/加速度），保存 EEPROM，并以 seq_ack 回 ACK。
 * - 执行 HOME 用 home_*，START(级联/随机) 用 work_*；未配置时用安全默认值。
 * - LED 不用于连接指示，仅保留随位置动效。
 * - 保留：级联/随机/测试、计次、限位回零、遥测与租约机制。
 *******************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <AccelStepper.h>
#include <Adafruit_NeoPixel.h>
#include <math.h>
#include <string.h>
#include <EEPROM.h>

/* ========= 组盐/标签（与主机一致） ========= */
#define GROUP_SALT16  0x45A2   // seq 高 16 位
#define TAG_NORMAL    0x5AA5   // 普通指令
#define TAG_PING      0xC001   // 发现/续约
#define TAG_ASSIGN    0xA55A   // 指派 ID
#define TAG_CONFIG    0xC0F7   // CONFIG 回执识别（主机用）

static inline uint16_t seq_hi(uint32_t s){ return (uint16_t)(s >> 16); }
static inline uint16_t seq_lo(uint32_t s){ return (uint16_t)(s & 0xFFFF); }
static inline bool valid_group(uint32_t s){ return seq_hi(s) == GROUP_SALT16; }

/* ========= 命令字（与主机一致） ========= */
#define CMD_START 1
#define CMD_STOP  2
#define CMD_PING  4
#define CMD_TEST  5
#define CMD_HOME  6
#define CMD_CFG   7  // ★ 一次性速度配置

/* ========= 可按需修改 ========= */
#define TARGET_CYCLES_DEFAULT 2   // 级联/随机每台默认往返次数

/* 硬件引脚（按实际接线修改） */
#define STEP_PIN    26
#define DIR_PIN     25
#define ENABLE_PIN1 27
#define ENABLE_PIN2  2
#define LIMIT_PIN   19
#define LIMIT_ACTIVE LOW

#define LED_PIN   18
#define LED_COUNT 64

/* 机械与运动（单位转换） */
const float STEPS_PER_CM = 15516.78f;
const float MAX_LENGTH_CM = 19.5f;
const float READY_POS_CM = 1.0f;

/* —— 当尚未收到 CONFIG 时的“安全默认速度” —— */
const float DEFAULT_HOME_SPEED = 8000.0f;   // 步/秒
const float DEFAULT_HOME_ACCEL = 2000.0f;   // 步/秒^2
const float DEFAULT_WORK_SPEED = 8000.0f;
const float DEFAULT_WORK_ACCEL = 2500.0f;

/* 级联模式最小外伸（用于计次判定） */
const float CYCLE_MIN_TRAVEL_CM = 5.0f;

/* LED 动效参数 */
#define LED_UPDATE_MS        100UL
#define LED_FADE_INTERVAL_MS 2000UL
#define LED_MIN_BRIGHTNESS   0
#define LED_MAX_BRIGHTNESS   255
#define LED_GAMMA            2.2f

/* MAC 表（固定设备 → 固定槽位），便于“按 MAC 认主/匹配槽位” */
static uint8_t SLAVE_MACS[6][6] = {
  {0xA0,0xA3,0xB3,0x2B,0x78,0x74},
  {0xA0,0xA3,0xB3,0x28,0x12,0x18},
  {0x08,0xD1,0xF9,0xD1,0xA3,0x20},
  {0xFC,0xF5,0xC4,0x4B,0xB9,0x5C},
  {0xA0,0xA3,0xB3,0x2B,0x45,0x0C},
  {0xA0,0xA3,0xB3,0x29,0x94,0x94}
};

/* ========= 发现窗口与租约 ========= */
#define DISCOVERY_WINDOW_MS  5000UL  // 上电后放宽接纳 PING/ASSIGN 的窗口
#define LEASE_TIMEOUT_MS     6000UL  // 长时间未收到主机包 → 退回发现窗口

/* ========= EEPROM 持久化 =========
 * 版本(1) + ID(1) + master_mac(6) + homeS(4) + homeA(4) + workS(4) + workA(4) = 1+1+6+16 = 24
 * 预留到 32 字节
 */
#define EEPROM_SIZE       32
#define EE_VER_OFFSET     0
#define EE_ID_OFFSET      1
#define EE_MAC_OFFSET     2   // 2..7
#define EE_HOME_S_OFFSET  8   // float
#define EE_HOME_A_OFFSET 12   // float
#define EE_WORK_S_OFFSET 16   // float
#define EE_WORK_A_OFFSET 20   // float
#define BIND_VERSION      2   // ★ 提升版本，包含速度配置

/* ====================================== */

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
Adafruit_NeoPixel leds(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

/* 协议结构体（与主机一致） */
struct __attribute__((packed)) ControlMsg {
  uint8_t  cmd;              // 1=START 2=STOP 4=Ping 5=Test 6=Home 7=Config
  uint8_t  target_id;
  float    start_cm;         // CONFIG: home_speed
  float    end_cm;           // CONFIG: home_accel
  float    trigger_cm;       // CONFIG: work_speed
  float    max_cm;           // CONFIG: work_accel
  float    speed;            // START/HOME 的临时速度（本实现默认忽略，使用保存配置）
  float    accel;            // START/HOME 的临时加速度（同上）
  uint8_t  mode;             // START: 0=级联 1=随机 2=测试
  uint32_t seq;              // [GROUP_SALT16 | TAG_*]
  uint16_t max_cycles;       // START: 次数
  uint16_t tele_interval_ms; // 遥测周期
};
struct __attribute__((packed)) TelemetryMsg {
  uint8_t  slave_id;         // 绑定前为 0；绑定后=固定 ID
  uint8_t  status;           // 0空闲 4触发 5OK 8限位 9测试完成 10循环完成
  float    pos_cm;
  uint32_t cycles;
  uint32_t seq_ack;          // 回 ACK：把收到的 seq 原样回主机
};

/* 运行状态机 */
enum SystemState { STATE_IDLE, STATE_RUNNING, STATE_HOMING_START, STATE_HOMING_TO_LIMIT, STATE_HOMING_TO_READY };
static SystemState system_state = STATE_IDLE;

/* 绑定/发现/租约 */
static uint8_t  SLAVE_ID = 0;
static uint8_t  master_mac[6] = {0};
static bool     master_known = false;
static uint32_t discovery_until_ms = 0;
static uint32_t last_master_rx_ms = 0;
static uint32_t g_tele_ms = 80;

/* 速度配置（来自 CONFIG；如未配置则用 DEFAULT_*） */
static bool  cfg_valid = false;
static float home_speed_cfg = DEFAULT_HOME_SPEED;
static float home_accel_cfg = DEFAULT_HOME_ACCEL;
static float work_speed_cfg = DEFAULT_WORK_SPEED;
static float work_accel_cfg = DEFAULT_WORK_ACCEL;

/* 模式与计次 */
static uint8_t  run_mode = 0;
static float    end_cm = MAX_LENGTH_CM;
static float    trigger_cm = 5.0f;
static bool     moving_forward = true;
static bool     trigger_reported = false;
static uint16_t max_cycles = TARGET_CYCLES_DEFAULT;
static uint32_t current_cycles = 0;
static bool     outward_reached_min = false;
static bool     at_zero_last = false;

static float    random_target_cm = 5.0f;
static bool     random_going_out = true;

static uint32_t last_tele_ms = 0;

/* LED 动效（不用于连接指示） */
static uint32_t last_led_update_ms = 0;
static uint32_t last_applied_color = 0;
static uint8_t  last_led_brightness = 0;
static float    led_cur_h = 0.0f, led_next_h = 120.0f;
static uint32_t led_fade_start_ms = 0;

/* 函数声明 */
static void onDataRecv(const uint8_t* mac, const uint8_t* data, int len);
static void onDataSent(const uint8_t* mac, esp_now_send_status_t status);

static void updateLEDByPos(float pos_cm);
static void checkCycleComplete();
static void startCascade();
static void startRandom();
static void startTest();

static bool addMasterPeer(const uint8_t* mac);
static void sendTelemetry(uint8_t status, uint32_t seq_ack = 0);

/* 工具 */
static long  cmToSteps(float cm){ return -(long)llroundf(cm * STEPS_PER_CM); }
static float stepsToCm(long steps){ return -(float)steps / STEPS_PER_CM; }
static float clampf(float x, float lo, float hi){ return (x<lo)?lo:((x>hi)?hi:x); }
static bool  macEquals(const uint8_t* a, const uint8_t* b){ for(int i=0;i<6;i++) if(a[i]!=b[i]) return false; return true; }

/* EEPROM：绑定 + 配置 读写 */
static void eepromLoadAll(){
  EEPROM.begin(EEPROM_SIZE);
  uint8_t ver = EEPROM.read(EE_VER_OFFSET);
  if (ver == BIND_VERSION){
    uint8_t id = EEPROM.read(EE_ID_OFFSET);
    uint8_t macbuf[6];
    for(int i=0;i<6;i++) macbuf[i]=EEPROM.read(EE_MAC_OFFSET+i);
    bool mac_nonzero=false; for(int i=0;i<6;i++){ if(macbuf[i]!=0){ mac_nonzero=true; break; } }

    float hs,ha,ws,wa;
    memcpy(&hs, (void*)(EEPROM.getDataPtr()+EE_HOME_S_OFFSET), sizeof(float));
    memcpy(&ha, (void*)(EEPROM.getDataPtr()+EE_HOME_A_OFFSET), sizeof(float));
    memcpy(&ws, (void*)(EEPROM.getDataPtr()+EE_WORK_S_OFFSET), sizeof(float));
    memcpy(&wa, (void*)(EEPROM.getDataPtr()+EE_WORK_A_OFFSET), sizeof(float));

    if (id>0 && mac_nonzero){
      SLAVE_ID = id;
      memcpy(master_mac, macbuf, 6);
      master_known = true; // 先认为已知主机，后续若收不到包会回发现窗口
    }
    // 配置有效性（>0）
    if (hs>0 && ha>0 && ws>0 && wa>0){
      home_speed_cfg = hs; home_accel_cfg = ha;
      work_speed_cfg = ws; work_accel_cfg = wa;
      cfg_valid = true;
    }
  }
}

static void eepromSaveBindingAndConfig(uint8_t id, const uint8_t* mac){
  EEPROM.write(EE_VER_OFFSET, BIND_VERSION);
  EEPROM.write(EE_ID_OFFSET, id);
  for(int i=0;i<6;i++) EEPROM.write(EE_MAC_OFFSET+i, mac[i]);
  // 配置也一起写（即使此刻可能尚未设置完，也写当前 RAM 值）
  memcpy((void*)(EEPROM.getDataPtr()+EE_HOME_S_OFFSET), &home_speed_cfg, sizeof(float));
  memcpy((void*)(EEPROM.getDataPtr()+EE_HOME_A_OFFSET), &home_accel_cfg, sizeof(float));
  memcpy((void*)(EEPROM.getDataPtr()+EE_WORK_S_OFFSET), &work_speed_cfg, sizeof(float));
  memcpy((void*)(EEPROM.getDataPtr()+EE_WORK_A_OFFSET), &work_accel_cfg, sizeof(float));
  EEPROM.commit();
}

static void eepromSaveConfigOnly(){
  // 假设版本/ID 已经存在
  memcpy((void*)(EEPROM.getDataPtr()+EE_HOME_S_OFFSET), &home_speed_cfg, sizeof(float));
  memcpy((void*)(EEPROM.getDataPtr()+EE_HOME_A_OFFSET), &home_accel_cfg, sizeof(float));
  memcpy((void*)(EEPROM.getDataPtr()+EE_WORK_S_OFFSET), &work_speed_cfg, sizeof(float));
  memcpy((void*)(EEPROM.getDataPtr()+EE_WORK_A_OFFSET), &work_accel_cfg, sizeof(float));
  EEPROM.commit();
}

/* ===== LED（动效；不用于连接状态） ===== */
static uint8_t mapBrightnessFromPos(float pos_cm){
  pos_cm = clampf(pos_cm, 0, MAX_LENGTH_CM);
  float x = pos_cm / MAX_LENGTH_CM;
  float y = powf(x, LED_GAMMA);
  float b = LED_MIN_BRIGHTNESS + y * (LED_MAX_BRIGHTNESS - LED_MIN_BRIGHTNESS);
  return (uint8_t)clampf(b + 0.5f, 0, 255);
}
static uint32_t hsvToColor(float h_deg, float s, float v){
  float h = fmodf(h_deg, 360.0f); if (h < 0) h += 360.0f;
  float c = v * s, x = c * (1 - fabsf(fmodf(h/60.0f, 2.0f) - 1)), m = v - c;
  float r,g,b;
  if (h < 60)      { r=c; g=x; b=0; }
  else if (h <120) { r=x; g=c; b=0; }
  else if (h <180) { r=0; g=c; b=x; }
  else if (h <240) { r=0; g=x; b=c; }
  else if (h <300) { r=x; g=0; b=c; }
  else             { r=c; g=0; b=x; }
  uint8_t R=(uint8_t)clampf((r+m)*255.0f + 0.5f, 0, 255);
  uint8_t G=(uint8_t)clampf((g+m)*255.0f + 0.5f, 0, 255);
  uint8_t B=(uint8_t)clampf((b+m)*255.0f + 0.5f, 0, 255);
  return leds.Color(R,G,B);
}
static void updateLEDByPos(float pos_cm){
  uint32_t now = millis();
  if (now - last_led_update_ms < LED_UPDATE_MS) return;
  last_led_update_ms = now;

  if (led_fade_start_ms == 0){
    led_fade_start_ms = now;
    led_cur_h = (float)(random(0, 360));
    led_next_h = fmodf(led_cur_h + 120.0f, 360.0f);
  }
  float u = (float)(now - led_fade_start_ms) / (float)LED_FADE_INTERVAL_MS;
  if (u >= 1.0f){
    led_cur_h = led_next_h;
    float delta = (float)random(60, 181);
    if (random(0,2)==0) delta = -delta;
    led_next_h = fmodf(led_cur_h + delta + 360.0f, 360.0f);
    led_fade_start_ms = now;
    u = 0.0f;
  }
  float dh = led_next_h - led_cur_h;
  if (dh > 180.0f) dh -= 360.0f;
  if (dh < -180.0f) dh += 360.0f;
  float h = led_cur_h + dh * u;
  if (h < 0.0f) h += 360.0f;
  if (h >= 360.0f) h -= 360.0f;

  uint8_t brightness = mapBrightnessFromPos(pos_cm);
  uint32_t color = hsvToColor(h, 1.0f, 1.0f);

  if (brightness != last_led_brightness || color != last_applied_color){
    leds.setBrightness(brightness);
    for (int i=0; i<LED_COUNT; ++i) leds.setPixelColor(i, color);
    leds.show();
    last_led_brightness = brightness;
    last_applied_color  = color;
  }
}

/* ===== 计次（级联模式用） ===== */
static void checkCycleComplete(){
  float pos = stepsToCm(stepper.currentPosition());
  bool at_zero = (fabsf(pos) < 0.5f);
  if (moving_forward && !outward_reached_min && pos >= CYCLE_MIN_TRAVEL_CM) outward_reached_min = true;
  if (at_zero && !at_zero_last && !moving_forward && outward_reached_min){
    current_cycles++;
    outward_reached_min = false;
    if (current_cycles >= max_cycles){
      stepper.stop();
      system_state = STATE_IDLE;
      sendTelemetry(10);
    }
  }
  at_zero_last = at_zero;
}

/* ===== 启动模式 ===== */
static void startCascade(){
  run_mode = 0;
  system_state = STATE_RUNNING;
  current_cycles = 0;
  moving_forward = true;
  trigger_reported = false;
  outward_reached_min = false;
  at_zero_last = false;
  // 用保存的 work 档
  stepper.setMaxSpeed(work_speed_cfg);
  stepper.setAcceleration(work_accel_cfg);
  stepper.moveTo(cmToSteps(end_cm));
}
static void startRandom(){
  run_mode = 1;
  system_state = STATE_RUNNING;
  current_cycles = 0;
  // 用保存的 work 档
  stepper.setMaxSpeed(work_speed_cfg);
  stepper.setAcceleration(work_accel_cfg);
  random_target_cm = (random(30, 191) / 10.0f); // 3.0~19.0 cm
  random_going_out = true;
  stepper.moveTo(cmToSteps(random_target_cm));
}
static void startTest(){
  run_mode = 2;
  system_state = STATE_RUNNING;
  current_cycles = 0;
  // 测试也用 work 档（你也可以改成 home 档）
  stepper.setMaxSpeed(work_speed_cfg);
  stepper.setAcceleration(work_accel_cfg);
  stepper.moveTo(cmToSteps(MAX_LENGTH_CM));
}

/* ===== Peer / Telemetry ===== */
static bool addMasterPeer(const uint8_t* mac){
  esp_now_peer_info_t p={};
  memcpy(p.peer_addr,mac,6);
  p.channel=1;
  p.encrypt=false;
  p.ifidx=WIFI_IF_STA;
  esp_err_t r=esp_now_add_peer(&p);
  return (r==ESP_OK || r==ESP_ERR_ESPNOW_EXIST);
}
static void sendTelemetry(uint8_t status, uint32_t seq_ack){
  if (!master_known) return;
  TelemetryMsg t;
  t.slave_id = SLAVE_ID;       // 未绑定时为 0
  t.status   = status;
  t.pos_cm   = stepsToCm(stepper.currentPosition());
  t.cycles   = current_cycles;
  t.seq_ack  = seq_ack;
  esp_err_t result = esp_now_send(master_mac, (uint8_t*)&t, sizeof(t));
  if (result == ESP_ERR_ESPNOW_NOT_FOUND) addMasterPeer(master_mac);
}

/* ===== 发现/绑定 + 指令接收 ===== */
static void learnMasterIfNeeded(const uint8_t* mac){
  if (!master_known){
    memcpy(master_mac, mac, 6);
    if (addMasterPeer(master_mac)) master_known=true;
  }
}

static void handleConfig(const ControlMsg* pc){
  // 仅当字段有效时更新（>0），否则保持原值
  if (pc->start_cm  > 0) home_speed_cfg = pc->start_cm;
  if (pc->end_cm    > 0) home_accel_cfg = pc->end_cm;
  if (pc->trigger_cm> 0) work_speed_cfg = pc->trigger_cm;
  if (pc->max_cm    > 0) work_accel_cfg = pc->max_cm;

  cfg_valid = (home_speed_cfg>0 && home_accel_cfg>0 && work_speed_cfg>0 && work_accel_cfg>0);

  // 应用到当前 stepper（不强行改正在运行中的目标，只更新配置）
  // 写入 EEPROM（与当前绑定一起存）
  eepromSaveConfigOnly();

  // ACK：原样回 seq
  sendTelemetry(5, pc->seq);
}

static void onDataRecv(const uint8_t* mac, const uint8_t* data, int len){
  if (len != (int)sizeof(ControlMsg)) return;
  const ControlMsg* pc = (const ControlMsg*)data;

  // 只接受本项目组盐
  if (!valid_group(pc->seq)) return;

  last_master_rx_ms = millis();
  uint16_t tag = seq_lo(pc->seq);

  // —— 发现阶段（只接受 PING / ASSIGN；若已绑定则可接受 CONFIG）——
  bool in_discovery = (millis() < discovery_until_ms) || (SLAVE_ID == 0) || !master_known;
  if (in_discovery){
    learnMasterIfNeeded(mac);

    // 指派 ID（优先）：保存绑定与当前配置
    if (tag == TAG_ASSIGN && pc->target_id > 0){
      SLAVE_ID = pc->target_id;
      eepromSaveBindingAndConfig(SLAVE_ID, mac);
      sendTelemetry(5, pc->seq); // ACK 绑定
      return;
    }

    // 若已绑定 ID，则允许 CONFIG（便于主机紧接着下发）
    if (pc->cmd == CMD_CFG && SLAVE_ID!=0 && pc->target_id==SLAVE_ID){
      handleConfig(pc);
      return;
    }

    // PING：回“通信正常”
    if (pc->cmd == CMD_PING){
      sendTelemetry(5, pc->seq);
      return;
    }

    // 其他命令在未绑定前不执行
    return;
  }

  // —— 严格模式：ID 必须匹配；若不匹配，仅允许 ASSIGN 纠正 —— //
  if (pc->target_id != SLAVE_ID) {
    if (tag == TAG_ASSIGN && pc->target_id > 0){
      SLAVE_ID = pc->target_id;
      eepromSaveBindingAndConfig(SLAVE_ID, mac);
      sendTelemetry(5, pc->seq);
    }
    return;
  }

  // 已绑定情况下，确认主机 peer
  learnMasterIfNeeded(mac);

  // CONFIG：一次性速度配置
  if (pc->cmd == CMD_CFG){
    handleConfig(pc);
    return;
  }

  // 遥测周期（可选）与临时速度（本实现默认忽略，使用保存配置）
  if (pc->tele_interval_ms >= 10) g_tele_ms = pc->tele_interval_ms;

  switch(pc->cmd){
    case CMD_START: {
      end_cm     = clampf(pc->end_cm, 0.0f, MAX_LENGTH_CM);
      trigger_cm = clampf(pc->trigger_cm, 0.0f, end_cm);
      max_cycles = (pc->max_cycles > 0) ? pc->max_cycles : TARGET_CYCLES_DEFAULT;

      if (pc->mode==0) startCascade();
      else if (pc->mode==1) startRandom();
      else if (pc->mode==2) startTest();
      break;
    }
    case CMD_STOP: {
      stepper.stop();
      system_state = STATE_IDLE;
      current_cycles = 0;
      outward_reached_min = false;
      at_zero_last = false;
      random_going_out = true;
      sendTelemetry(0, pc->seq);
      break;
    }
    case CMD_PING: {
      sendTelemetry(5, pc->seq);
      break;
    }
    case CMD_TEST: {
      startTest();
      break;
    }
    case CMD_HOME: {
      stepper.stop();
      system_state = STATE_HOMING_START;
      break;
    }
  }
}

static void onDataSent(const uint8_t* mac, esp_now_send_status_t status) {
  (void)mac; (void)status;
}

/* ===== 初始化 ===== */
void setup() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  pinMode(ENABLE_PIN1, OUTPUT);
  pinMode(ENABLE_PIN2, OUTPUT);
  digitalWrite(ENABLE_PIN1, LOW);   // 如驱动接法不同可对调
  digitalWrite(ENABLE_PIN2, HIGH);  // 如驱动接法不同可对调
  pinMode(LIMIT_PIN, INPUT_PULLUP);

  leds.begin();
  leds.clear();
  leds.setBrightness(50);
  leds.show(); // 不使用红/绿连接指示

  randomSeed((uint32_t)esp_random());

  // 给常见步进驱动一个更保险的 STEP 脉宽（微秒）
  stepper.setMinPulseWidth(2);

  // 读取 EEPROM 的绑定与速度配置
  eepromLoadAll();
  if (!master_known) {
    discovery_until_ms = millis() + DISCOVERY_WINDOW_MS;
  } else {
    discovery_until_ms = 0;
  }

  if (esp_now_init() != ESP_OK) {
    delay(3000);
    ESP.restart();
  }
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  system_state = STATE_IDLE;
}

/* ===== 主循环 ===== */
void loop() {
  // 若租约过期（长时间未收到主机消息），回到发现窗口
  if (master_known && (millis() - last_master_rx_ms > LEASE_TIMEOUT_MS)) {
    master_known = false;
    SLAVE_ID = 0; // 回到未绑定状态，等待主机重新指派
    discovery_until_ms = millis() + DISCOVERY_WINDOW_MS;
  }

  // LED 动效按位置更新
  updateLEDByPos(stepsToCm(stepper.currentPosition()));

  switch(system_state) {
    case STATE_IDLE:
      break;

    case STATE_HOMING_START: {
      // 使用保存的 home 档（若未配置则是默认值）
      stepper.setMaxSpeed(home_speed_cfg);
      stepper.setAcceleration(home_accel_cfg);
      if (digitalRead(LIMIT_PIN) == LIMIT_ACTIVE) {
        stepper.move(cmToSteps(2.0)); // 先移出约 2cm
        system_state = STATE_HOMING_TO_LIMIT;
      } else {
        stepper.move(cmToSteps(-50)); // 向限位移动足够距离（负方向）
        system_state = STATE_HOMING_TO_LIMIT;
      }
      break;
    }

    case STATE_HOMING_TO_LIMIT: {
      if (digitalRead(LIMIT_PIN) == LIMIT_ACTIVE) {
        stepper.stop();
        stepper.setCurrentPosition(0);
        delay(50);
        sendTelemetry(8); // 碰到限位
        // 从限位退到 READY，用 work/home 均可；此处用 work 档也完全没问题
        stepper.setMaxSpeed(work_speed_cfg);
        stepper.setAcceleration(work_accel_cfg);
        stepper.moveTo(cmToSteps(READY_POS_CM));
        system_state = STATE_HOMING_TO_READY;
      }
      break;
    }

    case STATE_HOMING_TO_READY: {
      if (stepper.distanceToGo() == 0) {
        sendTelemetry(0); // 空闲
        system_state = STATE_IDLE;
      }
      break;
    }

    case STATE_RUNNING: {
      if (run_mode == 0) { // 级联
        checkCycleComplete();
        if (system_state == STATE_IDLE) break;
      }
      if (stepper.distanceToGo() == 0) {
        if (run_mode == 0) {
          if (moving_forward) {
            moving_forward = false;
            stepper.moveTo(0);
          } else {
            moving_forward = true;
            trigger_reported = false;
            stepper.moveTo(cmToSteps(end_cm));
          }
          sendTelemetry(0);
        } else if (run_mode == 1) { // 随机
          if (random_going_out) {
            random_going_out = false;
            stepper.moveTo(0);
          } else {
            current_cycles++;
            if (current_cycles >= max_cycles) {
              stepper.stop();
              system_state = STATE_IDLE;
              sendTelemetry(10);
            } else {
              // 选择与上次至少差 1cm 的新目标
              float prev = random_target_cm;
              for (int k=0;k<10;k++){
                float v = (random(30, 191) / 10.0f);
                if (fabsf(v - prev) >= 1.0f) { random_target_cm = v; break; }
                if (k==9) random_target_cm = clampf(prev + ((prev < (3.0f+19.0f)/2)? +1.0f : -1.0f), 3.0f, 19.0f);
              }
              random_going_out = true;
              stepper.setMaxSpeed(work_speed_cfg);
              stepper.setAcceleration(work_accel_cfg);
              stepper.moveTo(cmToSteps(random_target_cm));
              sendTelemetry(0);
            }
          }
        } else if (run_mode == 2) { // 测试
          if (stepper.currentPosition() == cmToSteps(MAX_LENGTH_CM)) {
            stepper.moveTo(0);
          } else if (stepper.currentPosition() == 0) {
            system_state = STATE_IDLE;
            sendTelemetry(9);
          }
        }
      }
      break;
    }
  }

  if (system_state != STATE_IDLE) stepper.run();

  // 周期性遥测 + 级联触发点上报（仅供主机做链式触发参考）
  uint32_t now = millis();
  if (now - last_tele_ms >= g_tele_ms) {
    sendTelemetry(5);
    last_tele_ms = now;
  }
  if (system_state == STATE_RUNNING && run_mode == 0 && moving_forward && !trigger_reported) {
    float p = stepsToCm(stepper.currentPosition());
    if (p >= trigger_cm) {
      trigger_reported = true;
      sendTelemetry(4); // 触发点
    }
  }

}
