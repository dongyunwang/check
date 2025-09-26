# 项目重构计划 (Gemini) - V3 (Arduino版) 实现情况评估

## 评估结论概览

| 阶段 | 计划目标概述 | 代码现状 | 结论 |
| --- | --- | --- | --- |
| 阶段 1：从机单元功能验证 | 使用最小 `setup/loop` 架构，靠模拟指令驱动电机与 LED，自检全部核心动作 | 当前从机固件在 `setup()` 中直接初始化 ESP-NOW、LED、步进电机，并依赖回调接收主机指令；缺少计划中提到的 `current_command` / `new_command` 流程与模拟指令注入。 | ❌ 未满足 |
| 阶段 2：主从通信链路验证 | 移除模拟代码，通过 ESP-NOW 建立一次指令-遥测闭环；主机仅定时发送单一命令 | 从机仍包含完整展演逻辑，且 `setup()` 未区分“自测模式/网络模式”；主机实现了完整展演状态机而非计划要求的最简打点主机。 | ⚠️ 部分满足（存在超范围实现） |
| 阶段 3：完整多机协同逻辑 | 基于已验证的通信链路构建主机状态机，覆盖级联/随机展演 | 主机 `runExhibitionSequence()` 已实现级联与随机阶段切换、归位检测与循环运行；但由于阶段 1/2 的分层未落实，整体结构与计划差异较大。 | ✅ 功能覆盖但架构未按增量路线实现 |

> **结论**：代码最终功能更接近阶段 3 的目标，但没有按照计划的增量路径逐步实现和验证，难以复用计划所强调的“先单元逻辑、后通信、再整合”的开发流程。

## 关键差异详解

### 阶段 1：从机单元自测模式缺失
- 计划要求：
  - 在 `setup()` 中仅点亮红色 LED，`loop()` 暂留空白。 
  - 通过手动设置 `current_command` + `new_command` 来触发本地状态机，完成归位与运行动作。
- 实际代码：
  - `setup()` 内部直接初始化 Wi-Fi/ESP-NOW，并注册回调，LED 也在回调首次收到主机消息后切换为绿色，缺少单元测试模式的隔离。【F:slave_main.cpp†L82-L135】【F:slave_main.cpp†L362-L415】
  - 全局不存在 `current_command` 与 `new_command`，所有命令均来自 `onDataRecv()`；`loop()` 也已经运行完整的运行状态机而非预期的模拟触发流程。【F:slave_main.cpp†L294-L360】【F:slave_main.cpp†L393-L456】

### 阶段 2：通信链路验证与最简主机不匹配
- 计划要求：
  - 将从机切换为“网络模式”，`onDataRecv()` 负责填充全局命令；
  - 主机只需每隔 5 秒发送一次 `HOME`/`PING`，验证端到端通信。
- 实际代码：
  - 从机始终处于网络模式，没有保留与第一阶段之间的切换逻辑，导致无法单独运行单元测试版本。【F:slave_main.cpp†L300-L356】
  - 主机端实现了完整的展演流程：
    - `discoverAndVerifySlaves()` 执行握手与在线检测；
    - `runExhibitionSequence()` 包含归位、级联和随机阶段循环；
    - `runCascadePhase()` / `runRandomPhase()` 根据遥测推进状态机。【F:master_main.cpp†L142-L337】【F:master_main.cpp†L338-L471】
  - 尽管通信确实建立，但实现复杂度远超阶段 2 的最小需求。

### 阶段 3：多机协同逻辑
- 计划要求：构建状态机、维护从机状态数组、实现级联与随机运行。
- 实际代码：
  - 具备状态跟踪数组（例如 `device_completed`, `last_pos_cm`, `slave_responded` 等）。【F:master_main.cpp†L86-L124】
  - `onDataRecv()` 中调用 `updateCascadeTrigger()` 实现级联触发，`runRandomPhase()` 与 `runCascadePhase()` 负责阶段运行，整体逻辑与计划目标一致。【F:master_main.cpp†L209-L303】
  - 由于前两阶段未按计划拆分，虽然功能到位，但难以复用计划中的增量验证成果。

## 建议的后续任务
1. **重新实现阶段 1**：将当前从机固件拆分出纯自测版本，保留 LED 与电机状态机，但通过本地模拟命令驱动。
2. **实现可切换的运行模式**：在从机中新增配置或编译开关，以便在“自测模式”和“网络模式”间切换，满足阶段 1 ➔ 阶段 2 的过渡需求。
3. **编写最简主机样例**：在现有主机代码旁新增 `master_ping_demo.cpp`（或类似命名），实现阶段 2 要求的 5 秒打点逻辑，便于通信链路单独验证。
4. **补充开发文档**：记录三阶段编译/烧录方法与验证步骤，确保团队成员可以按照增量方案逐步测试。

## 当前固件的完整运行流程
1. **从机启动**：
   - `setup()` 初始化 Wi-Fi、ESP-NOW、步进电机及 LED。LED 默认红色，首次接收到主机消息后切换为绿色并进入常规位置指示模式。【F:slave_main.cpp†L362-L415】【F:slave_main.cpp†L316-L336】
   - 主循环 `loop()` 根据 `system_state` 执行归位、运行、随机等状态，同时定时发送遥测（状态码 5）并在触发点上报状态码 4。【F:slave_main.cpp†L393-L456】

2. **主机启动**：
   - `setup()` 初始化 ESP-NOW，调用 `discoverAndVerifySlaves()` 多次 PING 并等待所有已启用从机回应。【F:master_main.cpp†L338-L419】
   - 全部在线后进入 `runExhibitionSequence()`。

3. **归位阶段**：
   - `ensureHomeAll()` 依次下发停止与归位命令，并循环发送 PING，直到所有从机反馈位置接近就绪点或超时。【F:master_main.cpp†L142-L200】

4. **级联阶段**：
   - `runCascadePhase()` 启动参考从机（ID 由 `CASCADE_REF_DEVICE` 指定）。
   - 当参考从机累计位移达到 `CASCADE_TRIGGER_DISTANCE` 时，通过 `updateCascadeTrigger()` 触发下一台从机执行往返动作，直至所有从机完成设定次数并上报状态 10。【F:master_main.cpp†L204-L272】

5. **随机阶段**：
   - `runRandomPhase()` 同时启动所有启用的从机进入随机运动模式，每台设备完成设定次数后上报状态 10，主机循环等待全部完成。【F:master_main.cpp†L274-L317】

6. **循环执行**：
   - 阶段结束后调用 `stopAll()`，再次归位，然后根据配置重复级联与随机阶段，实现展览循环。【F:master_main.cpp†L319-L366】

