case CMD_HOME_E: return "HOME";
    case CMD_START_CAS: return "SCAS";
    case CMD_START_RND: return "SRND";
    case CMD_STOP_E: return "STOP";
    case CMD_TEST_E: return "TEST";
    case CMD_CONFIG_E: return "CONF";
    default: return "-";
  }
}
static const char* homeName(HomeProg h){
  switch(h){
    case HOME_SENT:  return "SENT";
    case HOME_LIMIT: return "LIMIT";
    case HOME_READY: return "READY";
    default:         return "-";
  }
}

static void printMac(const uint8_t* mac){
  for(int i=0;i<6;i++){
    Serial.printf("%02X", mac[i]);
    if (i<5) Serial.print(":");
  }
}

/* ====== 监控表 ====== */
static void printSlotHeader(){
  DBGLN("Slot | En | Bound | Onl | LastSeen(ms) | Status |   Pos  | Cycles | Cmd  | CmdAge(s) | Home  | Speed(H/W)");
}
static void printSlotLine(int i){
  float age_s = last_cmd_ms[i] ? (float)(millis() - last_cmd_ms[i]) / 1000.0f : -1.0f;
  Serial.printf(" #%%d  | %%c  | %%c     | %%c   | %%10lu | %%6u | %%6.2f | %%6lu | %%-4s | %%8.1f | %%-5s | \n",
    i+1,
    ENABLED[i]?'Y':'N',
    id_bound_ok[i]?'Y':'N',
    isOnline(i)?'Y':'N',
    (unsigned long)(millis() - last_seen_ms[i]),
    (unsigned)last_status[i],
    last_pos_cm[i],
    (unsigned long)device_cycles[i],
    cmdName(last_cmd_tag[i]),
    age_s,
    homeName(home_prog[i])
  );
  if (cfg_applied[i]){
    Serial.printf("H=%.0f/%.0f W=%.0f/%.0f\n",
      CFG_HOME_SPEED, CFG_HOME_ACCEL, CFG_WORK_SPEED, CFG_WORK_ACCEL);
  }else{
    Serial.println("-");
  }
}
static void monitorTick(bool force=false){
  if (!force && millis()-last_monitor_ms < MONITOR_PERIOD_MS) return;
  last_monitor_ms = millis();
  DBGLN("\n--- MONITOR ---");
  printSlotHeader();
  for(int i=0;i<NUM_SLAVES;i++){
    if (!ENABLED[i]) continue;
    printSlotLine(i);
  }
  DBGLN("--------------\n");
}

/* ====== ESP-NOW 基础 ====== */
static void onDataRecv(const uint8_t* mac, const uint8_t* data, int len);
static void onDataSent(const uint8_t* mac, esp_now_send_status_t status);
static bool initESPNow(){
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_ps(WIFI_PS_NONE);
  delay(50);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  delay(50);
  if (esp_now_init()!=ESP_OK) return false;
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);
  return true;
}
static bool addPeers(){
  for(int i=0;i<NUM_SLAVES;i++){
    if (!ENABLED[i]) continue;
    esp_now_peer_info_t p={};
    memcpy(p.peer_addr, SLAVE_MACS[i], 6);
    p.channel=WIFI_CHANNEL;
    p.encrypt=false;
    p.ifidx=WIFI_IF_STA;
    esp_now_add_peer(&p);
  }
  return true;
}

/* ====== 发指令（带标签） ====== */
static void sendCommand(uint8_t cmd, uint8_t slave_id, uint16_t tag,
                        float start_cm, float end_cm, float trig_cm, float max_cm,
                        float speed, float accel, uint8_t mode,
                        uint16_t max_cycles, uint16_t tele_ms)
{
  if (slave_id<1 || slave_id>NUM_SLAVES || !ENABLED[slave_id-1]) return;
  ControlMsg m={};
  m.cmd = cmd;
  m.target_id = slave_id;
  m.start_cm = start_cm;
  m.end_cm   = end_cm;
  m.trigger_cm = trig_cm;
  m.max_cm = max_cm;
  m.speed  = speed;
  m.accel  = accel;
  m.mode   = mode;
  m.seq    = make_seq(tag);
  m.max_cycles = max_cycles;
  m.tele_interval_ms = tele_ms;
  esp_now_send(SLAVE_MACS[slave_id-1], (uint8_t*)&m, sizeof(m));
}

/* 语义化封装 */
static inline void sendPing(uint8_t id, uint16_t tag=TAG_PING){
  sendCommand(CMD_PING, id, tag, 0,0,0,0, 0,0, 0, 0, EXHIBIT_TELE_INTERVAL_MS);
}
static inline void sendAssignID(uint8_t id){
  sendCommand(CMD_PING, id, TAG_ASSIGN, 0,0,0,0, 0,0, 0, 0, EXHIBIT_TELE_INTERVAL_MS);
  DBG("[ASSIGN] -> #%%u (try %%u)\n", id, assign_sent_count[id-1]+1);
}
static inline void sendStart(uint8_t id, uint8_t mode, uint16_t max_cycles){
  sendCommand(CMD_START, id, TAG_NORMAL, 0, DEFAULT_END_CM, DEFAULT_TRIGGER_CM, DEFAULT_END_CM,
              0,0, mode, max_cycles, EXHIBIT_TELE_INTERVAL_MS);
  DBG("[CMD] START #%%u mode=%%u cycles=%%u\n", id, mode, max_cycles);
  last_cmd_tag[id-1]  = (mode==0 ? CMD_START_CAS : (mode==1 ? CMD_START_RND : CMD_TEST_E));
  home_prog[id-1] = HOME_NONE;
  last_cmd_ms[id-1] = millis();
}
static inline void sendStop(uint8_t id){
  sendCommand(CMD_STOP, id, TAG_NORMAL, 0,0,0,0, 0,0, 0, 0, EXHIBIT_TELE_INTERVAL_MS);
  DBG("[CMD] STOP  #%%u\n", id);
  last_cmd_tag[id-1]  = CMD_STOP_E;
  last_cmd_ms[id-1] = millis();
}
static inline void sendHome(uint8_t id){
  sendCommand(CMD_HOME, id, TAG_NORMAL, 0,0,0,0, 0,0, 0, 0, EXHIBIT_TELE_INTERVAL_MS);
  DBG("[CMD] HOME  #%%u\n", id);
  last_cmd_tag[id-1]  = CMD_HOME_E;
  home_prog[id-1] = HOME_SENT;
  last_cmd_ms[id-1] = millis();
}
/* ★ 一次性速度配置：HOME/WORK 两套参数 */
static inline void sendConfig(uint8_t id){
  sendCommand(CMD_CFG, id, TAG_CONFIG,
              /*start=end as home*/   CFG_HOME_SPEED, CFG_HOME_ACCEL,
              /*trigger=max as work*/ CFG_WORK_SPEED, CFG_WORK_ACCEL,
              /*speed/accel*/ 0,0, 0, 0, EXHIBIT_TELE_INTERVAL_MS);
  DBG("[CFG] send -> #%%u  H=%.0f/%.0f  W=%.0f/%.0f\n",
      id, CFG_HOME_SPEED, CFG_HOME_ACCEL, CFG_WORK_SPEED, CFG_WORK_ACCEL);
  last_cmd_tag[id-1] = CMD_CONFIG_E;
  last_cmd_ms[id-1]  = millis();
}

/* ====== 绑定确认（阶段前小等） ====== */
static bool ensureBound(uint8_t id, uint32_t timeout_ms){
  int idx = id-1;
  if (!ENABLED[idx]) return true;
  if (id_bound_ok[idx]) return true;

  DBG("[BIND] Wait slot #%%u bound (<=%%lums)\n", id, (unsigned long)timeout_ms);
  uint32_t t0 = millis();
  while (millis()-t0 < timeout_ms){
    if (id_bound_ok[idx]) { DBG("[BIND] slot #%%u OK\n", id); return true; }
    uint32_t now = millis();
    if (assign_sent_count[idx] < ASSIGN_MAX_TRIES &&
        (now - last_assign_ms[idx] >= ASSIGN_RETRY_MS)){
      sendAssignID(id);
      assign_sent_count[idx]++;
      last_assign_ms[idx] = now;
    }
    sendPing(id);
    delay(60);
    monitorTick();
  }
  DBGLN("[WARN] slot #%%u not bound within %%lums, continue anyway", id, (unsigned long)timeout_ms);
  return false;
}
static void ensureBoundAll(uint32_t per_slot_timeout_ms){
  for(uint8_t i=1;i<=NUM_SLAVES;i++){
    if (!ENABLED[i-1]) continue;
    ensureBound(i, per_slot_timeout_ms);
  }
}

/* ====== CONFIG 下发与等待回执 ====== */
static bool ensureConfigAll(uint32_t wait_budget_ms){
  DBGLN("[CFG] Ensure all configured once");
  uint32_t t0 = millis();

  // 先发一轮未配置的
  for(uint8_t i=1;i<=NUM_SLAVES;i++){
    if (!ENABLED[i-1]) continue;
    if (!cfg_applied[i-1]){
      sendConfig(i);
      cfg_sent_count[i-1]++;
      last_cfg_ms[i-1] = millis();
      delay(15);
    }
  }

  while(true){
    bool all_cfg = true;
    for(int i=0;i<NUM_SLAVES;i++){
      if (!ENABLED[i]) continue;
      if (!cfg_applied[i]) { all_cfg=false; break; }
    }
    if (all_cfg) { DBGLN("[CFG] All configured"); return true; }
    if (millis() - t0 > wait_budget_ms){
      Serial.print("[CFG] Timeout waiting slots:");
      for(int i=0;i<NUM_SLAVES;i++) if (ENABLED[i] && !cfg_applied[i]) Serial.printf(" #%%d", i+1);
      Serial.println();
      return false;
    }
    // 重发未确认的
    for(uint8_t i=1;i<=NUM_SLAVES;i++){
      int idx=i-1; if (!ENABLED[idx] || cfg_applied[idx]) continue;
      if (cfg_sent_count[idx] < CONFIG_MAX_TRIES &&
          millis() - last_cfg_ms[idx] >= CONFIG_RESEND_MS){
        sendConfig(i);
        cfg_sent_count[idx]++;
        last_cfg_ms[idx]=millis();
      }
    }
    // 拉遥测
    for(uint8_t i=1;i<=NUM_SLAVES;i++){ if (ENABLED[i-1]){ sendPing(i); delay(20);} }
    monitorTick();
    delay(40);
  }
}

/* ====== 通用阶段工具 ====== */
static void resetCompletionStatus(){
  for(int i=0;i<NUM_SLAVES;i++){
    device_completed[i]=false;
    device_cycles[i]=0;
    slave_responded[i]=false;
    last_status[i]=0;
  }
}
static bool checkAllDevicesComplete(){
  for(int i=0;i<NUM_SLAVES;i++) if (ENABLED[i]) if (!device_completed[i]) return false;
  return true;
}

/* ====== 严格 HOME：全部 READY 才放行 ====== */
static void ensureHomeAll(){
  DBGLN("[PHASE] Ensure HOME (strict): STOP -> HOME -> wait ALL READY");

  // 先确保绑定
  ensureBoundAll(BOUND_WAIT_MS_PER_SLOT);

  // 再确保一次性速度配置（只要没 ACK 就会重发/等待一会）
  ensureConfigAll(CONFIG_WAIT_BUDGET_MS);

  // STOP -> HOME
  for(uint8_t i=1;i<=NUM_SLAVES;i++) if (ENABLED[i-1]){ sendStop(i); delay(15); }
  delay(250);
  for(uint8_t i=1;i<=NUM_SLAVES;i++) if (ENABLED[i-1]){ sendHome(i); delay(15); }

  uint32_t last_ping_ms   = 0;
  uint32_t last_report_ms = 0;

  while(true){
    bool all_ready = true;
    for(int i=0;i<NUM_SLAVES;i++){
      if (!ENABLED[i]) continue;
      if (home_prog[i] != HOME_READY){ all_ready = false; break; }
    }
    if (all_ready){
      DBGLN("[PHASE] HOME ALL READY");
      break;
    }

    if (millis() - last_ping_ms > PING_INTERVAL_MS){
      last_ping_ms = millis();
      for(uint8_t i=1;i<=NUM_SLAVES;i++){
        if (!ENABLED[i-1]) continue;
        sendPing(i);
        delay(30);
      }
    }

    for(int i=0;i<NUM_SLAVES;i++){
      if (!ENABLED[i]) continue;
      if (home_prog[i] == HOME_READY) continue;
      if (millis() - last_cmd_ms[i] >= HOME_RESEND_MS){
        sendHome(i+1);
        DBG("[HOME] re-issue -> #%%d\n", i+1);
      }
    }

    if (millis() - last_report_ms > HOME_GATE_LOG_EVERY){
      last_report_ms = millis();
      Serial.print("[HOME] waiting:");
      for(int i=0;i<NUM_SLAVES;i++){
        if (!ENABLED[i]) continue;
        if (home_prog[i] != HOME_READY){
          Serial.printf(" #%%d(%%s)", i+1, homeName(home_prog[i]));
        }
      }
      Serial.println();
      monitorTick(true);
    }
    delay(10);
  }
}

/* ====== 级联触发链 ====== */
static void updateCascadeTrigger(float ref_pos){
  static const float CASCADE_TRIGGER_DISTANCE = 5.0f;
  float delta=fabsf(ref_pos-device_last_pos);
  device_accumulated_distance += delta;
  device_last_pos = ref_pos;
  if (device_accumulated_distance >= CASCADE_TRIGGER_DISTANCE){
    for(int i=1;i<NUM_SLAVES;i++){
      if (ENABLED[i] && !cascade_device_started[i]){
        // 触发前尽量确保绑定
        ensureBound(i+1, 800);
        sendStart((uint8_t)(i+1), 0, TARGET_CYCLES_CASCADE);
        cascade_device_started[i]=true;
        device_accumulated_distance -= CASCADE_TRIGGER_DISTANCE;
        DBG("[CASCADE] Trigger -> #%%d (accum=%.2f)\n", i+1, device_accumulated_distance);
        break;
      }
    }
  }
}

/* ====== 接收遥测 ====== */
static void onDataRecv(const uint8_t* mac, const uint8_t* data, int len){
  if (len!=(int)sizeof(TelemetryMsg)) return;
  TelemetryMsg t;
  memcpy(&t,data,sizeof(t));

  int idx = macIndex(mac);
  if (idx<0) {
    Serial.print("[RX] From unknown MAC "); printMac(mac); Serial.println();
    return;
  }
  if (!ENABLED[idx]) return;

  uint8_t expected_id = (uint8_t)(idx+1);

  last_pos_cm[idx] = t.pos_cm;
  last_seen_ms[idx]= millis();
  device_cycles[idx]= t.cycles;
  slave_responded[idx] = true;
  last_status[idx] = t.status;

  // 简要日志（不刷屏）
  // Serial.print("[RX] MAC "); printMac(mac);
  // Serial.printf(" -> slot #%%u, rep_id=%%u, st=%%u, pos=%.2f, cyc=%lu, ack=0x%08lX\n",
  //               expected_id, t.slave_id, t.status, t.pos_cm, (unsigned long)t.cycles,
  //               (unsigned long)t.seq_ack);

  // ★ CONFIG 回执
  if (t.seq_ack == CONFIG_SEQ && !cfg_applied[idx]){
    cfg_applied[idx] = true;
    DBGLN("[CFG] ACK from slot #%%u", expected_id);
  }

  // 归零进度推断
  if (last_cmd_tag[idx] == CMD_HOME_E) {
    if (t.status == 8) {
      if (home_prog[idx] != HOME_LIMIT) {
        home_prog[idx] = HOME_LIMIT;
        DBG("[HOME] slot #%%u -> LIMIT (%.2fcm)\n", expected_id, t.pos_cm);
      }
    } else if (t.status == 0) {
      bool at_ready = (fabsf(t.pos_cm - READY_POS_CM) <= READY_TOL_CM);
      if (at_ready && home_prog[idx] != HOME_READY) {
        home_prog[idx] = HOME_READY;
        DBG("[HOME] slot #%%u -> READY (%.2fcm)\n", expected_id, t.pos_cm);
      }
    }
  }

  // 绑定：若上报ID不等于期望ID，则触发指派
  if (t.slave_id != expected_id){
    uint32_t now = millis();
    if (!id_bound_ok[idx] && assign_sent_count[idx] < ASSIGN_MAX_TRIES &&
        (now - last_assign_ms[idx] >= ASSIGN_RETRY_MS)){
      sendAssignID(expected_id);
      assign_sent_count[idx]++;
      last_assign_ms[idx] = now;
    }
  } else if (!id_bound_ok[idx]) {
    id_bound_ok[idx] = true;
    DBG("[BIND] slot #%%u bound OK\n", expected_id);
  }

  // 级联触发：参考槽位
  if (system_running && current_phase==PHASE_CASCADE && expected_id==1){
    updateCascadeTrigger(t.pos_cm);
  }

  // 完成判定
  if (t.status==10){
    device_completed[idx]=true;
    DBG("[DONE] slot #%%u completed this phase\n", expected_id);
  }

  monitorTick();
}

static void onDataSent(const uint8_t* mac, esp_now_send_status_t status){
  (void)mac; (void)status;
}

/* ====== 预热与探活 ====== */
static void preheatBroadcast(uint8_t rounds=3){
  DBGLN("[BOOT] Preheat PING x%%u", rounds);
  for(uint8_t r=0;r<rounds;r++){
    for(uint8_t j=1;j<=NUM_SLAVES;j++){
      if (!ENABLED[j-1]) continue;
      sendPing(j);
      delay(18);
    }
  }
  DBGLN("[BOOT] Preheat done");
}

static bool discoverAndVerifySlaves(){
  DBGLN("[DISCOVER] Probe %%d enabled slots", enabledCount());
  for(int r=0;r<3;r++){
    for(uint8_t j=1;j<=NUM_SLAVES;j++) if (ENABLED[j-1]) { sendPing(j); delay(18); }
  }
  DBGLN("[DISCOVER] Wait 2s for responses...");
  delay(2000);
  int found=0;
  for(int i=0;i<NUM_SLAVES;i++){
    if (ENABLED[i] && slave_responded[i]){ found++; Serial.printf(" - Slot #%%d responded\n", i+1); }
    else if (ENABLED[i]){ Serial.printf(" - Slot #%%d NO response\n", i+1); }
  }
  DBGLN("[DISCOVER] Found %%d / %%d enabled", found, enabledCount());
  return true;
}

/* ====== 入口 ====== */
void setup(){
  Serial.begin(115200);
  Serial.println("\n=== Master (Plan B) + CONFIG(CMD=7) + Monitor + Strict HOME Gate ===");

  if (!initESPNow()){
    Serial.println("ESP-NOW init failed. Restarting...");
    delay(2500); ESP.restart();
  }
  addPeers();

  printMacTable();
  preheatBroadcast(3);
  discoverAndVerifySlaves();

  Serial.println("Entering exhibition sequence...");
  runExhibitionSequence(); // 不返回
}

void loop(){
  delay(10);
}
