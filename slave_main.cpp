/*******************************************************
 * Gemini-修改日志 (累计)
 *
 * --- 版本 2.1 ---
 * 日期: 2025-09-19
 * 变更内容:
 *  - 在 loop() 中增加了一个条件，以防止正常的LED更新逻辑覆盖启动时的红/绿状态指示灯。
 *
 * --- 版本 2 ---
 * 日期: 2025-09-19
 * 变更内容:
 *  - 添加了连接状态的视觉反馈功能。
 *    - 启动时，LED亮红灯表示“等待主机”。
 *    - 收到主机的第一个消息后，LED变绿灯以确认“连接已建立”。
 *
 * --- 版本 1 ---
 * 日期: 2025-09-19
 * 变更内容:
 *  - 修改了 setup() 以防止上电时自动归位。从机现在以静默的IDLE状态启动，等待主机指令。
 *    这是实现健壮的“握手”启动协议的一部分。
 *******************************************************/

/*******************************************************
 * ESP32 Slave (精简版：去串口输出，仅保留参数注释位)
 * 有中文注释的地方为可改动的地方。
 * 9.18 20.52 稳定
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

/* ========= 需要用户按需修改的参数 ========= */
#define TARGET_CYCLES_DEFAULT  2     // ← 默认次数（级联/随机模式每台的目标往返次数）

/* 硬件引脚（按实际接线修改） */
#define STEP_PIN      26            // ← 步进脉冲引脚
#define DIR_PIN       25            // ← 方向引脚
#define ENABLE_PIN1   27            // ← 使能引脚1（根据驱动接法调整高低）
#define ENABLE_PIN2   2             // ← 使能引脚2（根据驱动接法调整高低）
#define LIMIT_PIN     19            // ← 机械限位开关引脚
#define LIMIT_ACTIVE  LOW           // ← 限位触发电平（常见为 LOW）

#define LED_PIN       18            // ← NeoPixel 数据引脚
#define LED_COUNT     64            // ← 灯珠数量（8x8 面板为 64）

/* 机械与运动（按丝杆/减速比/期望速度修改） */
const float STEPS_PER_CM    = 15516.78f;  // ← 1cm 对应的步数（按丝杆螺距/细分标定）
const float MAX_LENGTH_CM   = 19.5f;      // ← 可用行程上限
const float HOME_SPEED      = 5000.0f;    // ← 回零速度
const float HOME_ACCEL      = 500.0f;     // ← 回零加速度
const float WORK_SPEED      = 4000.0f;    // ← 工作速度（级联/随机）
const float WORK_ACCEL      = 800.0f;     // ← 工作加速度
const float READY_POS_CM    = 1.0f;       // ← 回零后退至的就绪位置

/* 级联模式最小外伸（用于计次判定） */
const float CYCLE_MIN_TRAVEL_CM = 5.0f;   // ← 至少外伸到此才计一次“往返”

/* LED 参数（按视觉效果调整） */
#define LED_UPDATE_MS         100UL       // ← LED 刷新间隔（ms）
#define LED_FADE_INTERVAL_MS  2000UL      // ← 完成一次色相渐变的时长（ms）
#define LED_MIN_BRIGHTNESS    0
#define LED_MAX_BRIGHTNESS    255
#define LED_GAMMA             2.2f

/* MAC 表（同一固件自动识别从机ID，必要时替换为你自己的6台 MAC） */
static uint8_t SLAVE_MACS[6][6] = {
  {0xA0,0xA3,0xB3,0x2B,0x78,0x74},
  {0xA0,0xA3,0xB3,0x28,0x12,0x18},
  {0x08,0xD1,0xF9,0xD1,0xA3,0x20},
  {0xFC,0xF5,0xC4,0x4B,0xB9,0x5C},
  {0xA0,0xA3,0xB3,0x2B,0x45,0x0C},
  {0xA0,0xA3,0xB3,0x29,0x94,0x94}
};
/* ====================================== */

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
Adafruit_NeoPixel leds(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

struct __attribute__((packed)) ControlMsg {
  uint8_t  cmd;             // 1=启动 2=停止 4=Ping 5=测试 6=归位
  uint8_t  target_id;
  float    start_cm;
  float    end_cm;
  float    trigger_cm;
  float    max_cm;
  float    speed;
  float    accel;
  uint8_t  mode;            // 0=级联 1=随机 2=测试
  uint32_t seq;
  uint16_t max_cycles;
  uint16_t tele_interval_ms;
};

struct __attribute__((packed)) TelemetryMsg {
  uint8_t  slave_id;
  uint8_t  status;          // 0=空闲 4=已触发 5=通信正常 8=碰到限位 9=测试完成 10=循环完成
  float    pos_cm;
  uint32_t cycles;
  uint32_t seq_ack;
};

enum SystemState { STATE_IDLE, STATE_RUNNING, STATE_HOMING_START, STATE_HOMING_TO_LIMIT, STATE_HOMING_TO_READY };
static SystemState system_state = STATE_IDLE;
static bool visual_feedback_connected = false;

static uint8_t SLAVE_ID = 0;
static uint8_t master_mac[6] = {0};
static bool    master_known = false;

static uint8_t run_mode     = 0;
static float   end_cm       = MAX_LENGTH_CM;
static float   trigger_cm   = 5.0f;

static bool  moving_forward = true;
static bool  trigger_reported = false;

static uint16_t max_cycles        = TARGET_CYCLES_DEFAULT;
static uint32_t current_cycles    = 0;
static bool     outward_reached_min = false;
static bool     at_zero_last      = false;

static float random_target_cm = 5.0f;
static bool  random_going_out = true;

static uint32_t g_tele_ms = 80;
static uint32_t last_tele_ms = 0;

static uint32_t last_led_update_ms   = 0;
static uint32_t last_applied_color   = 0;
static uint8_t  last_led_brightness  = 0;
static float    led_cur_h = 0.0f, led_next_h = 120.0f;
static uint32_t led_fade_start_ms = 0;
static uint32_t led_pause_until_ms = 0;
static uint8_t  led_marquee_index = 0;
static uint8_t  led_last_lit_pixels = 0;

static void onDataRecv(const uint8_t* mac, const uint8_t* data, int len);
static void onDataSent(const uint8_t* mac, esp_now_send_status_t status);
static void updateLEDByPos(float pos_cm);
static bool runToCmBlocking(float cm_target, uint32_t timeout_ms = 8000);
static void checkCycleComplete();
static void startCascade();
static void startRandom();
static void startTest();
static bool addMasterPeer(const uint8_t* mac);
static void sendTelemetry(uint8_t status, uint32_t seq_ack = 0);
static uint8_t getMyID();
static float generateRandomLength();
static float generateRandomLengthDifferent(float prev, float min_diff=1.0f);

static long cmToSteps(float cm){ return -(long)llroundf(cm * STEPS_PER_CM); }
static float stepsToCm(long steps){ return -(float)steps / STEPS_PER_CM; }
static float clampf(float x, float lo, float hi){ return (x<lo)?lo:((x>hi)?hi:x); }

static uint8_t getMyID(){
  uint8_t mac[6]; WiFi.macAddress(mac);
  for (int i=0;i<6;i++){
    bool match=true;
    for (int j=0;j<6;j++){ if (SLAVE_MACS[i][j]!=mac[j]){ match=false; break; } }
    if (match) return (uint8_t)(i+1);
  }
  return 0;
}

static float generateRandomLength() { return random(30, 191) / 10.0f; }
static float generateRandomLengthDifferent(float prev, float min_diff){
  for (int k=0; k<10; ++k){
    float v = generateRandomLength();
    if (fabsf(v - prev) >= min_diff) return v;
  }
  float v = prev + ((prev < ((3.0f+19.0f)/2)) ? +min_diff : -min_diff);
  return clampf(v, 3.0f, 19.0f);
}

static bool addMasterPeer(const uint8_t* mac){
  esp_now_peer_info_t p={}; memcpy(p.peer_addr,mac,6);
  p.channel=1; p.encrypt=false; p.ifidx=WIFI_IF_STA;
  esp_err_t r=esp_now_add_peer(&p);
  return (r==ESP_OK || r==ESP_ERR_ESPNOW_EXIST);
}

static void sendTelemetry(uint8_t status, uint32_t seq_ack){
  if (!master_known) return;
  TelemetryMsg t;
  t.slave_id = SLAVE_ID;
  t.status   = status;
  t.pos_cm   = stepsToCm(stepper.currentPosition());
  t.cycles   = current_cycles;
  t.seq_ack  = seq_ack;
  esp_err_t result = esp_now_send(master_mac, (uint8_t*)&t, sizeof(t));
  if (result == ESP_ERR_ESPNOW_NOT_FOUND) addMasterPeer(master_mac);
}

/* ===== LED ===== */
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
  if      (h < 60)  { r=c; g=x; b=0; }
  else if (h < 120) { r=x; g=c; b=0; }
  else if (h < 180) { r=0; g=c; b=x; }
  else if (h < 240) { r=0; g=x; b=c; }
  else if (h < 300) { r=x; g=0; b=c; }
  else              { r=c; g=0; b=x; }
  uint8_t R = (uint8_t)clampf((r+m)*255.0f + 0.5f, 0, 255);
  uint8_t G = (uint8_t)clampf((g+m)*255.0f + 0.5f, 0, 255);
  uint8_t B = (uint8_t)clampf((b+m)*255.0f + 0.5f, 0, 255);
  return leds.Color(R,G,B);
}

static void updateLEDByPos(float pos_cm){
  uint32_t now = millis();

  if (led_pause_until_ms != 0) {
    if ((int32_t)(now - led_pause_until_ms) < 0) {
      return; // 仍处于主机握手阶段，保持纯绿色
    }
    led_pause_until_ms = 0;
  }

  if (now - last_led_update_ms < LED_UPDATE_MS) return;
  last_led_update_ms = now;

  if (led_fade_start_ms == 0){
    led_fade_start_ms = now;
    led_cur_h  = (float)(random(0, 360));
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
  if (dh > 180.0f)  dh -= 360.0f;
  if (dh < -180.0f) dh += 360.0f;
  float h = led_cur_h + dh * u;
  if (h < 0.0f) h += 360.0f;
  if (h >= 360.0f) h -= 360.0f;

  float progress = clampf(pos_cm / MAX_LENGTH_CM, 0.0f, 1.0f);
  uint8_t pixels_to_light = (uint8_t)clampf(roundf(progress * LED_COUNT), 0.0f, (float)LED_COUNT);

  if (pixels_to_light == 0) {
    leds.clear();
    leds.show();
    led_marquee_index = 0;
    led_last_lit_pixels = 0;
    last_applied_color = 0;
    last_led_brightness = 0;
    return;
  }

  if (pixels_to_light != led_last_lit_pixels) {
    if (pixels_to_light == 0) {
      led_marquee_index = 0;
    } else {
      led_marquee_index %= pixels_to_light;
    }
    led_last_lit_pixels = pixels_to_light;
  }

  uint8_t brightness = mapBrightnessFromPos(pos_cm);
  leds.setBrightness(brightness);

  uint32_t base_color = hsvToColor(h, 1.0f, 0.7f);
  uint8_t base_r = (uint8_t)((base_color >> 16) & 0xFF);
  uint8_t base_g = (uint8_t)((base_color >> 8) & 0xFF);
  uint8_t base_b = (uint8_t)(base_color & 0xFF);
  uint32_t head_color = hsvToColor(h, 1.0f, 1.0f);

  for (uint8_t i = 0; i < LED_COUNT; ++i) {
    if (i < pixels_to_light) {
      if (i == led_marquee_index) {
        leds.setPixelColor(i, head_color);
      } else {
        leds.setPixelColor(i, leds.Color(base_r, base_g, base_b));
      }
    } else {
      leds.setPixelColor(i, 0);
    }
  }

  leds.show();
  led_marquee_index = (led_marquee_index + 1) % pixels_to_light;
  last_led_brightness = brightness;
  last_applied_color  = base_color;
}

/* ===== 仅自检用的阻塞移动 ===== */
static bool runToCmBlocking(float cm_target, uint32_t timeout_ms){
  stepper.moveTo(cmToSteps(cm_target));
  uint32_t t0 = millis();
  while (stepper.distanceToGo() != 0) {
    stepper.run();
    updateLEDByPos(stepsToCm(stepper.currentPosition()));
    if (millis() - t0 > timeout_ms) return false;
    if (digitalRead(LIMIT_PIN) == LIMIT_ACTIVE && cm_target > 0.1f){ stepper.stop(); return false; }
  }
  return true;
}

/* ===== 级联计次 ===== */
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
  run_mode = 0; system_state = STATE_RUNNING; current_cycles = 0;
  moving_forward = true; trigger_reported = false; outward_reached_min = false; at_zero_last = false;
  stepper.setMaxSpeed(WORK_SPEED); stepper.setAcceleration(WORK_ACCEL);
  stepper.moveTo(cmToSteps(end_cm));
}
static void startRandom(){
  run_mode = 1; system_state = STATE_RUNNING; current_cycles = 0;
  stepper.setMaxSpeed(WORK_SPEED); stepper.setAcceleration(WORK_ACCEL);
  random_target_cm = generateRandomLengthDifferent(random_target_cm, 1.0f);
  random_going_out = true; stepper.moveTo(cmToSteps(random_target_cm));
}
static void startTest(){
  run_mode = 2; system_state = STATE_RUNNING; current_cycles = 0;
  stepper.setMaxSpeed(WORK_SPEED); stepper.setAcceleration(WORK_ACCEL);
  stepper.moveTo(cmToSteps(MAX_LENGTH_CM));
}

/* ===== ESP-NOW 回调 ===== */
static void onDataRecv(const uint8_t* mac, const uint8_t* data, int len){
  if (len != (int)sizeof(ControlMsg)) return;
  ControlMsg c; memcpy(&c,data,sizeof(c));
  if (SLAVE_ID==0 || c.target_id!=SLAVE_ID) return;

  // 视觉反馈：第一次收到主机消息时变为绿色
  if (!visual_feedback_connected) {
    visual_feedback_connected = true;
    leds.setBrightness(50);
    for(int i=0; i<LED_COUNT; i++) {
      leds.setPixelColor(i, leds.Color(0, 255, 0)); // 设置为绿色
    }
    leds.show();
    led_pause_until_ms = millis() + 2000; // 握手成功后维持绿色2秒
    led_marquee_index = 0;
  }

  if (!master_known){ memcpy(master_mac, mac, 6); if (addMasterPeer(master_mac)) master_known=true; }
  if (c.tele_interval_ms >= 10) g_tele_ms = c.tele_interval_ms;
  if (c.speed > 0)  stepper.setMaxSpeed(c.speed);
  if (c.accel > 0)  stepper.setAcceleration(c.accel);

  switch(c.cmd){
    case 1: {
      end_cm     = clampf(c.end_cm, 0.0f, MAX_LENGTH_CM);
      trigger_cm = clampf(c.trigger_cm, 0.0f, end_cm);
      max_cycles = (c.max_cycles > 0) ? c.max_cycles : TARGET_CYCLES_DEFAULT;
      if (c.mode==0)      startCascade();
      else if (c.mode==1) startRandom();
      else if (c.mode==2) startTest();
      break;
    }
    case 2: {
      stepper.stop(); system_state=STATE_IDLE;
      current_cycles = 0; outward_reached_min=false; at_zero_last=false; random_going_out = true;
      sendTelemetry(0);
      break;
    }
    case 4: { sendTelemetry(5, c.seq); break; } // Ping请求
    case 5: { startTest(); break; }             // 测试
    case 6: {                                   // 归位
      stepper.stop(); system_state = STATE_HOMING_START;
      break;
    }
  }
}
static void onDataSent(const uint8_t* mac, esp_now_send_status_t status){}

// 步进电机控制任务，将运行在核心0上
void stepperTask(void *pvParameters) {
  // Serial.println("Stepper task started on core 0"); // 调试时可取消注释
  for (;;) { // 无限循环
    // 持续调用 stepper.run()，这是唯一需要做的事
    stepper.run();

    // 给予其他在核心0上运行的低优先级任务（如WiFi）一点点时间
    // vTaskDelay(1) 会让出CPU至少1个tick（通常是1ms）
    vTaskDelay(1); 
  }
}

/* ===== 初始化 ===== */
void setup(){
  WiFi.mode(WIFI_STA); WiFi.disconnect(); esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_promiscuous(true); esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE); esp_wifi_set_promiscuous(false);

  pinMode(ENABLE_PIN1, OUTPUT); pinMode(ENABLE_PIN2, OUTPUT);
  digitalWrite(ENABLE_PIN1, LOW);           // ← 如驱动接法不同可对调
  digitalWrite(ENABLE_PIN2, HIGH);          // ← 如驱动接法不同可对调
  pinMode(LIMIT_PIN, INPUT_PULLUP);

  leds.begin();
  leds.clear();
  leds.setBrightness(50); // 使用中等亮度作为状态指示
  for(int i=0; i<LED_COUNT; i++) {
    leds.setPixelColor(i, leds.Color(255, 0, 0)); // 设置为红色
  }
  leds.show();
  randomSeed((uint32_t)esp_random());

  SLAVE_ID = getMyID();

  if (esp_now_init()!=ESP_OK){ delay(3000); ESP.restart(); }
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  system_state = STATE_IDLE;  // 上电后保持静默，等待主机指令

  // 创建并启动步进电机任务，并将其“钉”在核心0上
  xTaskCreatePinnedToCore(
      stepperTask,      // 1. 要运行的函数
      "StepperTask",    // 2. 任务名（用于调试）
      4096,             // 3. 任务的堆栈大小（字节）
      NULL,             // 4. 传递给任务的参数（这里不需要）
      1,                // 5. 任务优先级（0是最低，数字越大优先级越高）
      NULL,             // 6. 任务句柄（这里不需要）
      0                 // 7. 钉在哪个核心上（0或1）。我们将它钉在核心0
  );
}

/* ===== 主循环 ===== */
void loop(){
  // 仅当从机已被主机联络后，才运行基于位置的常规LED更新逻辑。
  // 否则，LED将保持其红/绿状态。
  if (visual_feedback_connected) {
    updateLEDByPos(stepsToCm(stepper.currentPosition()));
  }

  switch(system_state) {
    case STATE_IDLE:
      delay(2);
      return;

    case STATE_HOMING_START:
      stepper.setMaxSpeed(HOME_SPEED);
      stepper.setAcceleration(HOME_ACCEL);
      if (digitalRead(LIMIT_PIN) == LIMIT_ACTIVE) {
          stepper.move(cmToSteps(2.0));  // 先移出2cm
          system_state = STATE_HOMING_TO_LIMIT;
      } else {
          stepper.move(cmToSteps(-50));  // 向限位移动足够距离
          system_state = STATE_HOMING_TO_LIMIT;
      }
      break;

    case STATE_HOMING_TO_LIMIT:
      if (digitalRead(LIMIT_PIN) == LIMIT_ACTIVE) {
        stepper.stop();
        stepper.setCurrentPosition(0);
        delay(50);
        sendTelemetry(8);                 // 碰到限位
        stepper.setMaxSpeed(WORK_SPEED);
        stepper.setAcceleration(WORK_ACCEL);
        stepper.moveTo(cmToSteps(READY_POS_CM));
        system_state = STATE_HOMING_TO_READY;
      }
      break;

    case STATE_HOMING_TO_READY:
      if (stepper.distanceToGo() == 0) {
        sendTelemetry(0);                 // 空闲
        system_state = STATE_IDLE;
      }
      break;

    case STATE_RUNNING:
      if (run_mode==0){
        checkCycleComplete();
        if (system_state == STATE_IDLE) return;
      }

      if (stepper.distanceToGo()==0){
        if (run_mode==0){ // 级联
          if (moving_forward){ moving_forward=false; stepper.moveTo(0); }
          else { moving_forward=true; trigger_reported=false; stepper.moveTo(cmToSteps(end_cm)); }
          sendTelemetry(0);
        }
        else if (run_mode==1){ // 随机
          if (random_going_out) {
            random_going_out = false;
            stepper.moveTo(0);
          } else {
            current_cycles++;
            if (current_cycles >= max_cycles){
              stepper.stop(); system_state = STATE_IDLE;
              sendTelemetry(10);
            } else {
              random_target_cm = generateRandomLengthDifferent(random_target_cm, 1.0f);
              random_going_out = true;
              stepper.setMaxSpeed(WORK_SPEED);
              stepper.setAcceleration(WORK_ACCEL);
              stepper.moveTo(cmToSteps(random_target_cm));
              sendTelemetry(0);
            }
          }
        }
        else if (run_mode==2){ // 测试
          if (stepper.currentPosition()==cmToSteps(MAX_LENGTH_CM)){ stepper.moveTo(0); }
          else if (stepper.currentPosition()==0){ system_state=STATE_IDLE; sendTelemetry(9); }
        }
      }
      break;
  }

  // if (system_state != STATE_IDLE) stepper.run(); // 此行已由后台任务取代，必须移除

  if (system_state == STATE_RUNNING) {
    uint32_t now=millis();
    if (now-last_tele_ms>=g_tele_ms){ sendTelemetry(5); last_tele_ms=now; }
    if (run_mode==0 && moving_forward && !trigger_reported){
      float p=stepsToCm(stepper.currentPosition());
      if (p>=trigger_cm){ trigger_reported=true; sendTelemetry(4); }
    }
  }
}