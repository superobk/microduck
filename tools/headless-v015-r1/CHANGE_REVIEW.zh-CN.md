# 逐项代码评审：headless-v015-r1

基线：`a9ec4b2079ef8ee7904014089c885bb07d57d63c`。下列为47条修改规则（其中一条作用于3处），以及3个新增Rust文件。所有规则源自固定源码锚点；本环境没有运行完整仓库应用/编译验证。

## 新增模块的职责

`morphology.rs` 是不可变构型与数据映射。`initialize_from_env` 在打开串口前加载，`motor_ids/active_slots` 唯一定义设备成员，`project_sensors` 仅替换缺失槽，`project_command` 设置固定头颈命令，`pack_positions` 原顺序打包，`thermal_summary` 排除虚拟温度。

`headless.rs` 把物理构型转换成daemon能力限制；`refusal` 覆盖请求/通知，`enforce_stop` 在总线所属线程锁止和关闭扭矩，`response` 只提供本地诊断与排队复位。重复IMU值默认只保留诊断，因为量化后的静止数据也可能重复。

`robotd/tests/headless_profile.rs` 在单独进程运行真实daemon的fake后端，避免OnceLock/静态故障状态在测试间互相污染。它不调用真实串口，不加载策略。

## 始终不变的契约

15逻辑槽、嘴槽9、14原始动作、61观测、原HOME、原policy loader、原ONNX字节、原动作历史语义。真实总线只移除30–33，嘴34由独立开关决定。配置未设置时保持Full15。

## 每条修改规则

# `duck-control/src/lib.rs`

固定原文件Git blob：`ca6f95df16fb2224289b6e49ec4665b363930931`。

## 01. export morphology module

作用次数：1。**原因：**在既有控制库导出构型模块，让总线、控制器和daemon使用同一份只读配置；不修改模型常量。

修改前：

```rust
pub mod model;
pub mod obs;
```

修改后：

```rust
pub mod model;
pub mod morphology;
pub mod obs;
```

# `duck-control/src/bus.rs`

固定原文件Git blob：`083ab4821681cfc027584d6a97e74c51d3b30195`。

## 02. import physical topology

作用次数：1。**原因：**总线引入构型类型；真实设备集合由这一配置决定，不由某次读失败推测。

修改前：

```rust
use crate::imu::{IMU_BLOCK_LEN, SflpDecoder};
```

修改后：

```rust
use crate::imu::{IMU_BLOCK_LEN, SflpDecoder};
use crate::morphology::{self, Morphology};
```

## 03. persist physical ID to logical-slot mapping

作用次数：1。**原因：**持久保存真实电机ID及其原15槽位置。不能在读取返回后直接紧凑枚举到15槽。

修改前：

```rust
    ids: Vec<u8>,
```

修改后：

```rust
    ids: Vec<u8>,
    morphology: &'static Morphology,
    motor_ids: Vec<u8>,
    joint_slots: Vec<usize>,
```

## 04. construct only expected physical IDs

作用次数：1。**原因：**组合读取仍首先读真实IMU ID200，再读取真实电机；头颈ID不进入任何待响应列表。

修改前：

```rust
        let mut ids = Vec::with_capacity(NUM_JOINTS + 1);
        ids.push(IMU_DXL_ID);
        ids.extend_from_slice(&JOINT_IDS);
```

修改后：

```rust
        let morphology = morphology::current();
        let joint_slots: Vec<usize> = morphology.active_slots().collect();
        let motor_ids = morphology.motor_ids();
        let mut ids = Vec::with_capacity(motor_ids.len() + 1);
        ids.push(IMU_DXL_ID);
        ids.extend_from_slice(&motor_ids);
```

## 05. store mapping in IO instance

作用次数：1。**原因：**打开串口时固定成员表和槽位映射，后续重连/读写共享一致的物理顺序。

修改前：

```rust
            ids,
            fast_sync_read,
            imu: SflpDecoder::default(),
```

修改后：

```rust
            ids,
            morphology,
            motor_ids,
            joint_slots,
            fast_sync_read,
            imu: SflpDecoder::default(),
```

## 06. register checks cover physical motors only without borrow aliasing

作用次数：1。**原因：**寄存器检查仅覆盖真实电机；启动阶段克隆ID表以避免遍历self字段与可变方法调用之间的Rust借用冲突。

修改前：

```rust
        for &id in &JOINT_IDS {
            fixed += self.check_registers_of(id)?;
        }
```

修改后：

```rust
        for id in self.motor_ids.clone() {
            fixed += self.check_registers_of(id)?;
        }
```

## 07. ping torque gain target physical list

作用次数：3。**原因：**同一替换规则作用于缺失检测、扭矩和增益3处循环。若只改高频读写，这些路径仍会等待头颈ID或在中途失败。

修改前：

```rust
        for &id in &JOINT_IDS {
```

修改后：

```rust
        for &id in &self.motor_ids {
```

## 08. retain input-voltage shutdown protection in headless builds

作用次数：1。**原因：**只在缺失头颈模式保留输入电压故障位，52按位或1成为53。明确区分安全寄存器调整与不变的ONNX权重；这不放宽电压规格。

修改前：

```rust
        for &(name, want) in EXPECTED_REGISTERS {
            // rustypot
```

修改后：

```rust
        for &(name, want) in EXPECTED_REGISTERS {
            // The fixed-head prototype does not inherit the full robot's deliberate
            // disabling of the input-voltage shutdown bit. This changes no policy data.
            let want = if self.morphology.headless() && name == "shutdown" { want | 1 } else { want };
            // rustypot
```

## 09. never adopt a replacement into a deliberately missing slot

作用次数：1。**原因：**新增电机自动采用逻辑只能补真实成员中的缺口，不能把出厂ID1误分配成一个明确不存在的头颈关节。

修改前：

```rust
    pub fn adopt_replacement(&mut self, id: u8) -> Result<bool> {
        let name = JOINT_IDS
```

修改后：

```rust
    pub fn adopt_replacement(&mut self, id: u8) -> Result<bool> {
        if !self.motor_ids.contains(&id) {
            return Err(IoError::Bus(format!("{id} is not a configured physical servo")));
        }
        let name = JOINT_IDS
```

## 10. boot positions scatter rather than positional copy

作用次数：1。**原因：**启动读回10/11个位置，按joint_slots写入原15槽；缺失槽先填固定角。完整15模式仍读回全部原顺序。

修改前：

```rust
    pub fn present_positions(&mut self) -> Result<[f64; NUM_JOINTS]> {
        let values = self
            .controller
            .sync_read_present_position(&JOINT_IDS)
            .map_err(|e| IoError::Bus(format!("read present positions: {e}")))?;
        if values.len() != NUM_JOINTS {
            return Err(IoError::ShortRead {
                what: "present positions",
                expected: NUM_JOINTS,
                got: values.len(),
            });
        }
        let mut out = [0.0; NUM_JOINTS];
        out.copy_from_slice(&values);
        Ok(out)
    }
```

修改后：

```rust
    pub fn present_positions(&mut self) -> Result<[f64; NUM_JOINTS]> {
        let values = self
            .controller
            .sync_read_present_position(&self.motor_ids)
            .map_err(|e| IoError::Bus(format!("read present positions: {e}")))?;
        if values.len() != self.motor_ids.len() {
            return Err(IoError::ShortRead {
                what: "present positions",
                expected: self.motor_ids.len(),
                got: values.len(),
            });
        }
        let mut out = self.morphology.seed_positions();
        for (&joint, value) in self.joint_slots.iter().zip(values) { out[joint] = value; }
        Ok(out)
    }
```

## 11. bench profile cannot enable torque

作用次数：1。**原因：**在真实总线扭矩开启函数再次检查allow_motion，台架配置不能因遗漏高层检查而使能扭矩。

修改前：

```rust
    pub fn set_torque(&mut self, on: bool) -> Result<()> {
        let mut failed = Vec::new();
```

修改后：

```rust
    pub fn set_torque(&mut self, on: bool) -> Result<()> {
        if on && !self.morphology.motion_allowed() {
            return Err(IoError::Bus("bench-only morphology: allow_motion is false".to_owned()));
        }
        let mut failed = Vec::new();
```

## 12. initialize synthetic slots before decoding physical samples

作用次数：1。**原因：**先填充构型规定的固定虚拟状态，再写入真实数据；不把未安装关节伪装成已经读到正常反馈。

修改前：

```rust
        let mut sensors = Sensors::default();

        // Slot 0 is the IMU board.
```

修改后：

```rust
        let mut sensors = Sensors::default();
        self.morphology.project_sensors(&mut sensors);

        // Slot 0 is the IMU board.
```

## 13. map compressed bus block indexes back to logical indexes

作用次数：1。**原因：**同步读取包已不包含头颈，因此返回块的第5/6个电机不再是原逻辑第5/6槽。使用joint_slots把右腿放回10..14；原解码和单位转换不变。

修改前：

```rust
        for (joint, block) in blocks[1..].iter().enumerate() {
```

修改后：

```rust
        for (&joint, block) in self.joint_slots.iter().zip(&blocks[1..]) {
```

## 14. send only actual physical target prefix

作用次数：1。**原因：**先将15槽目标按真实成员顺序打包，再与相同motor_ids一起发送；不发送缺失ID，也不错误地取action[:10]。

修改前：

```rust
    fn write(&mut self, targets: &JointTargets) -> Result<()> {
        self.controller
            .sync_write_goal_position(&JOINT_IDS, &targets.positions)
            .map_err(|e| IoError::Bus(format!("sync_write goal positions: {e}")))
    }
```

修改后：

```rust
    fn write(&mut self, targets: &JointTargets) -> Result<()> {
        let (packed, count) = self.morphology.pack_positions(&targets.positions);
        debug_assert_eq!(count, self.motor_ids.len());
        self.controller
            .sync_write_goal_position(&self.motor_ids, &packed[..count])
            .map_err(|e| IoError::Bus(format!("sync_write goal positions: {e}")))
    }
```

## 15. explicit absent servo reboot is an error

作用次数：1。**原因：**明确指定不存在的舵机时拒绝，而不是向这个ID发送重启并伪报成功；“全部重启”的入口另外修改。

修改前：

```rust
    fn reboot(&mut self, id: u8) -> Result<()> {
        // The status packet
```

修改后：

```rust
    fn reboot(&mut self, id: u8) -> Result<()> {
        if self.morphology.is_absent_id(id) {
            return Err(IoError::Bus(format!("servo {id} is deliberately absent in this morphology")));
        }
        // The status packet
```

## 16. slow read physical ID list

作用次数：1。**原因：**温度与电压也必须只读真实成员。高频已适配而慢速仍读15台，会留下周期性故障。

修改前：

```rust
.sync_read_raw_data(&JOINT_IDS, SLOW_READ_ADDR, SLOW_READ_LEN)
```

修改后：

```rust
.sync_read_raw_data(&self.motor_ids, SLOW_READ_ADDR, SLOW_READ_LEN)
```

## 17. slow read validates active count

作用次数：1。**原因：**长度检查按实际成员数量计算，仍严格拒绝真实成员缺失或短包，不将错误补成正常数据。

修改前：

```rust
        if blocks.len() != NUM_JOINTS {
            return Err(IoError::ShortRead {
                what: "voltage+temperature blocks",
                expected: NUM_JOINTS,
```

修改后：

```rust
        if blocks.len() != self.motor_ids.len() {
            return Err(IoError::ShortRead {
                what: "voltage+temperature blocks",
                expected: self.motor_ids.len(),
```

## 18. slow sensors scatter back to logical slots

作用次数：1。**原因：**保留原温度数组槽位，避免右腿温度被归到头部或错误的关节名。

修改前：

```rust
        for (joint, block) in blocks.iter().enumerate() {
```

修改后：

```rust
        for (&joint, block) in self.joint_slots.iter().zip(&blocks) {
```

## 19. empty voltage error refers to expected physical count

作用次数：1。**原因：**错误消息中的预期数应是实际10/11成员，不把缺失头部算作本应返回电压的设备。

修改前：

```rust
                what: "input voltage",
                expected: NUM_JOINTS,
```

修改后：

```rust
                what: "input voltage",
                expected: self.motor_ids.len(),
```

# `robotd/src/control.rs`

固定原文件Git blob：`a02ec53e517bc30e0993b2a68760266d36147043`。

## 20. disable seated boot and shutdown sit selection

作用次数：1。**原因：**让缺失头颈机器不进入整机坐下/起身流程，即使原始网络文件仍存在。

修改前：

```rust
    pub fn has_sitstand(&self) -> bool {
        self.policy.has_sitstand()
    }
```

修改后：

```rust
    pub fn has_sitstand(&self) -> bool {
        !duck_control::morphology::current().headless() && self.policy.has_sitstand()
    }
```

## 21. defensive whole-body skill refusal: pub fn start_ground_pick(&mut self) -> Result<(), &'static str> {

作用次数：1。**原因：**在控制器内部也拒绝这一整机技能，作为IPC拒绝之后的第二道限制，避免内部调用绕过入口。

修改前：

```rust
    pub fn start_ground_pick(&mut self) -> Result<(), &'static str> {
```

修改后：

```rust
    pub fn start_ground_pick(&mut self) -> Result<(), &'static str> {
        if duck_control::morphology::current().headless() {
            return Err("whole-body skills disabled by headless morphology");
        }
```

## 22. defensive whole-body skill refusal: pub fn start_skill(&mut self, index: usize) -> Result<bool, &'static str> {

作用次数：1。**原因：**在控制器内部也拒绝这一整机技能，作为IPC拒绝之后的第二道限制，避免内部调用绕过入口。

修改前：

```rust
    pub fn start_skill(&mut self, index: usize) -> Result<bool, &'static str> {
```

修改后：

```rust
    pub fn start_skill(&mut self, index: usize) -> Result<bool, &'static str> {
        if duck_control::morphology::current().headless() {
            return Err("whole-body skills disabled by headless morphology");
        }
```

## 23. defensive whole-body skill refusal: pub fn sit_toggle(&mut self) -> Result<&'static str, &'static str> {

作用次数：1。**原因：**在控制器内部也拒绝这一整机技能，作为IPC拒绝之后的第二道限制，避免内部调用绕过入口。

修改前：

```rust
    pub fn sit_toggle(&mut self) -> Result<&'static str, &'static str> {
```

修改后：

```rust
    pub fn sit_toggle(&mut self) -> Result<&'static str, &'static str> {
        if duck_control::morphology::current().headless() {
            return Err("whole-body skills disabled by headless morphology");
        }
```

## 24. do not advertise unavailable controller skills

作用次数：1。**原因：**控制器对外的可执行技能列表为空，避免按钮仍展示整机技能为可用。

修改前：

```rust
    pub fn skill_names(&self) -> Vec<String> {
        self.skills
```

修改后：

```rust
    pub fn skill_names(&self) -> Vec<String> {
        if duck_control::morphology::current().headless() { return Vec::new(); }
        self.skills
```

## 25. defensive internal skill guard pub fn begin_shutdown_sit(&mut self) {

作用次数：1。**原因：**内部开机/关机调用不能绕过限制触发整机坐起；完整15模式则保持原有行为。

修改前：

```rust
    pub fn begin_shutdown_sit(&mut self) {
```

修改后：

```rust
    pub fn begin_shutdown_sit(&mut self) {
        if duck_control::morphology::current().headless() { return; }
```

## 26. defensive internal skill guard pub fn begin_boot_rise(&mut self) {

作用次数：1。**原因：**内部开机/关机调用不能绕过限制触发整机坐起；完整15模式则保持原有行为。

修改前：

```rust
    pub fn begin_boot_rise(&mut self) {
```

修改后：

```rust
    pub fn begin_boot_rise(&mut self) {
        if duck_control::morphology::current().headless() { return; }
```

## 27. build virtual observation in any backend and clear unsupported modes

作用次数：1。**原因：**在推理入口再次投影状态，防止fake或其他路径绕过物理补位；清除不支持的技能状态，但不改正常ONNX状态/历史接口。

修改前：

```rust
    ) -> Result<Step, PolicyError> {
        // Expire windows first
```

修改后：

```rust
    ) -> Result<Step, PolicyError> {
        let morphology = duck_control::morphology::current();
        let mut projected = *sensors;
        morphology.project_sensors(&mut projected);
        let sensors = &projected;
        let body_active = body_active && !morphology.headless();
        if morphology.headless() {
            self.active = None;
            self.ground_pick = None;
            self.sit = Sit::Up;
        }
        // Expire windows first
```

## 28. effective command is projected before ONNX

作用次数：1。**原因：**有效命令需可变，以便在技能选择之后、观测构建之前设置与固定结构一致的命令。

修改前：

```rust
        let (net, effective, label) = if let Some(active) = self.active {
```

修改后：

```rust
        let (net, mut effective, label) = if let Some(active) = self.active {
```

## 29. fixed-head command uses lock minus HOME

作用次数：1。**原因：**有效头颈目标必须与固定角一致，且保持原命令是相对HOME偏移的语义；不能把旧摇杆头部目标继续送入网络。

修改前：

```rust
        self.last_net = Some(net);

        let observation = Observation::build(
```

修改后：

```rust
        morphology.project_command(&mut effective);
        self.last_net = Some(net);

        let observation = Observation::build(
```

## 30. fixed head targets after normal filtering; raw action history unchanged

作用次数：1。**原因：**保留旧缩放/滤波顺序，再固定无执行器目标。原始14维action历史已经保存，不被执行掩码改写。

修改前：

```rust
        self.previous = Some(targets);

        // Advance the windows
```

修改后：

```rust
        morphology.project_positions(&mut targets);
        self.previous = Some(targets);

        // Advance the windows
```

# `robotd/src/main.rs`

固定原文件Git blob：`ab574b662e5b1e27b63dbbb6773609cfdd73297e`。

## 31. daemon side capability module

作用次数：1。**原因：**安全/命令门禁属于daemon而不是纯控制库，避免将进程/socket依赖塞进duck-control。

修改前：

```rust
mod control;
mod intents;
```

修改后：

```rust
mod control;
mod headless;
mod intents;
```

## 32. load independent hardware sidecar before params and IO

作用次数：1。**原因：**在任何总线初始化前校验构型；指定错误文件直接失败，不尝试15舵机默认。启动日志输出实际成员用于验收。

修改前：

```rust
    let explicit = args.params.is_some();
```

修改后：

```rust
    if let Err(e) = duck_control::morphology::initialize_from_env() {
        tracing::error!(error = %e, "bad morphology; no motor bus has been opened");
        return ExitCode::FAILURE;
    }
    tracing::warn!(report = %duck_control::morphology::current().report(), "physical morphology");

    let explicit = args.params.is_some();
```

## 33. forbid wrong startup mode and automatic limp recovery in headless profile

作用次数：1。**原因：**不允许轮滑模式或未经改造的完整机体sim；关闭原limp后自动交还起身网络的路径，使用手动复位锁止。台架配置禁止独立init上扭矩。

修改前：

```rust
    if let Some(Command::Init { duration }) = args.command {
        // init opens
```

修改后：

```rust
    if duck_control::morphology::current().headless() {
        if args.sim.is_some() {
            tracing::error!("headless --sim needs a matching locked-body simulator; use --fake for software-only tests");
            return ExitCode::FAILURE;
        }
        if params.policy.mode != Mode::Walk {
            tracing::error!("headless profile requires policy.mode=walk");
            return ExitCode::FAILURE;
        }
        // Replace the full-body limp -> automatic rise handoff with an explicit latch.
        params.safety.limp_fall = false;
    }
    if let Some(Command::Init { duration }) = args.command {
        if !duck_control::morphology::current().motion_allowed() {
            tracing::error!("bench-only morphology refuses init: allow_motion is false");
            return ExitCode::FAILURE;
        }
        // init opens
```

## 34. synthetic slots in fake as well as hardware telemetry

作用次数：1。**原因：**即使fake后端，也将缺失槽明确显示成固定虚拟状态。真实腿部和IMU不被覆盖。

修改前：

```rust
        let fresh = match safety.read() {
            Ok(sensors) => {
                state.consecutive_errors.store(0, Ordering::Relaxed);
```

修改后：

```rust
        let fresh = match safety.read() {
            Ok(mut sensors) => {
                duck_control::morphology::current().project_sensors(&mut sensors);
                state.consecutive_errors.store(0, Ordering::Relaxed);
```

## 35. latch falls and repeated sensor faults before any enable decision

作用次数：1。**原因：**在获取本帧命令快照之前处理故障，防止同帧待处理init或enable把扭矩重新打开；总线拥有线程负责执行关闭与重试。

修改前：

```rust
        state.fallen.store(safety.fallen(), Ordering::Relaxed);

        let snapshot = intents.snapshot();
```

修改后：

```rust
        state.fallen.store(safety.fallen(), Ordering::Relaxed);

        if headless::enforce_stop(&mut safety, &state, &intents, fresh.as_ref()) {
            bringup = Bringup::Limp;
            was_driving = false;
            hold = coast.known_positions(hold);
            if let Some(c) = controller.as_mut() { c.reset(); }
        }
        let snapshot = intents.snapshot();
```

## 36. reboot all means all physical motors

作用次数：1。**原因：**“重启全部”必须由构型枚举真实ID；否则即使底层拒绝缺失ID，这个高层列表仍会报错或中途停止。

修改前：

```rust
            let ids: Vec<u8> = if ids.is_empty() {
                duck_control::model::JOINT_IDS.to_vec()
```

修改后：

```rust
            let ids: Vec<u8> = if ids.is_empty() {
                duck_control::morphology::current().motor_ids()
```

## 37. report and infer the same sanitized velocity command

作用次数：1。**原因：**观测与遥测使用同一个限幅后的命令，避免状态声称某个速度而策略实际收到另一个速度。

修改前：

```rust
        let (mut targets, gain, moving, policy_label) = match (driving, sensors.as_ref()) {
```

修改后：

```rust
        let mut command = command;
        duck_control::morphology::current().project_command(&mut command);
        let (mut targets, gain, moving, policy_label) = match (driving, sensors.as_ref()) {
```

## 38. final projection also covers homing hold mouth and non-policy paths

作用次数：1。**原因：**在所有目标生产者之后、Safety之前再次固定虚拟槽，覆盖HOME插值、保持、嘴部声音联动等非策略路径。

修改前：

```rust
        match safety.apply(targets, hold, gain) {
```

修改后：

```rust
        duck_control::morphology::current().project_positions(&mut targets);
        duck_control::morphology::current().project_positions(&mut hold);
        match safety.apply(targets, hold, gain) {
```

## 39. temperature summary excludes synthetic values

作用次数：1。**原因：**温度极值、均值和最热关节名只使用真实成员，虚拟零值不参与统计。

修改前：

```rust
            let (hottest, max_c) = slow.temps_c.iter().enumerate().fold(
                (0usize, f64::MIN),
                |(best, high), (joint, &t)| {
                    if t > high { (joint, t) } else { (best, high) }
                },
            );
            let mean_c = slow.temps_c.iter().sum::<f64>() / slow.temps_c.len() as f64;
```

修改后：

```rust
            let (hottest, max_c, mean_c) =
                duck_control::morphology::current().thermal_summary(&slow.temps_c);
```

## 40. local diagnostics without modifying IPC enum

作用次数：1。**原因：**在原Call拒绝未知方法后扩展两个仅本地使用的诊断方法，保持旧共享协议枚举与客户端二进制不变。

修改前：

```rust
        let call = request.as_call();

        // Notifications get no reply
```

修改后：

```rust
        let call = request.as_call();
        // Ordinary calls keep the original decoder and fast path. Decode an envelope
        // only for an unknown/invalid call; do not depend on private Request fields.
        if call.is_err() {
            let envelope = serde_json::to_value(&request).unwrap_or(serde_json::Value::Null);
            let method = envelope.get("method").and_then(serde_json::Value::as_str).unwrap_or("");
            if envelope.get("jsonrpc").and_then(serde_json::Value::as_str) == Some("2.0")
                && matches!(method, "robot.morphology" | "robot.morphology.rearm")
            {
                if let Some(id) = request.id.clone() {
                    let empty_params = envelope.get("params").is_none_or(|p| {
                        p.is_null() || p.as_object().is_some_and(|o| o.is_empty())
                    });
                    let response = if empty_params {
                        headless::response(&state, &intents, id, method)
                    } else {
                        proto::Response::err(Some(id), proto::Error::new(
                            proto::code::INVALID_PARAMS, "morphology methods accept no parameters"))
                    };
                    write_line(&mut write_half, &response).await?;
                }
                continue;
            }
        }

        // Notifications get no reply
```

## 41. notification path capability gate

作用次数：1。**原因：**手柄连续输入常用没有id的JSON-RPC通知；只处理有id请求的拒绝并不完整。

修改前：

```rust
fn apply_intent(state: &RobotState, intents: &Intents, call: &proto::Call) -> bool {
    match call {
```

修改后：

```rust
fn apply_intent(state: &RobotState, intents: &Intents, call: &proto::Call) -> bool {
    if headless::refusal(state, intents, call).is_some() { return false; }
    match call {
```

## 42. do listing contains no unsupported skills

作用次数：1。**原因：**提供给技能名称校验/按钮能力的列表也必须与实际执行权限一致。

修改前：

```rust
fn do_names(policies: &PolicyNames) -> Vec<String> {
    let mut names
```

修改后：

```rust
fn do_names(policies: &PolicyNames) -> Vec<String> {
    if duck_control::morphology::current().headless() { return Vec::new(); }
    let mut names
```

## 43. skill queue capability gate

作用次数：1。**原因：**即便其他调用路径绕过显示层，任务排队入口也不允许整机技能进入控制循环。

修改前：

```rust
fn queue_skill(state: &RobotState, intents: &Intents, skill: &str) -> bool {
    let policies
```

修改后：

```rust
fn queue_skill(state: &RobotState, intents: &Intents, skill: &str) -> bool {
    if duck_control::morphology::current().headless() { return false; }
    let policies
```

## 44. request path returns refusal instead of accepted-but-silent

作用次数：1。**原因：**有id请求明确返回拒绝及原因。robot.look原本返回LookResult，因此使用JSON-RPC错误，避免返回错误形状的成功结果。

修改前：

```rust
    call: &proto::Call,
) -> proto::Response {
    match call {
```

修改后：

```rust
    call: &proto::Call,
) -> proto::Response {
    if let Some(reason) = headless::refusal(state, intents, call) {
        // robot.look normally returns LookResult, not IntentResult: use a JSON-RPC
        // error rather than an incorrectly shaped successful result for that method.
        if matches!(call, proto::Call::RobotLook(_)) {
            return proto::Response::err(Some(id), proto::Error::new(proto::code::INVALID_PARAMS, reason));
        }
        return proto::Response::ok(Some(id), &proto::IntentResult::refused(reason));
    }
    match call {
```

## 45. capability listing excludes every whole-body skill

作用次数：1。**原因：**robot.skills不再从旧配置表列出本构型无法执行的动作；并不删除磁盘上的权重。

修改前：

```rust
fn skills_report(state: &RobotState) -> proto::SkillsResult {
    let configured
```

修改后：

```rust
fn skills_report(state: &RobotState) -> proto::SkillsResult {
    if duck_control::morphology::current().headless() {
        return proto::SkillsResult { skills: Vec::new(), built_in: Vec::new() };
    }
    let configured
```

## 46. subscription does not advertise disabled skills

作用次数：1。**原因：**旧客户端通过订阅确认能力时不会收到可用sitstand/ground_pick/skill列表，避免UI误导。

修改前：

```rust
                    sitstand: policies.sitstand.clone(),
                    ground_pick: policies.ground_pick.clone(),
                    skills: policies.skills.clone(),
```

修改后：

```rust
                    sitstand: if duck_control::morphology::current().headless() { None } else { policies.sitstand.clone() },
                    ground_pick: if duck_control::morphology::current().headless() { None } else { policies.ground_pick.clone() },
                    skills: if duck_control::morphology::current().headless() { Vec::new() } else { policies.skills.clone() },
```

## 47. policy report differentiates loaded slots from executable skills

作用次数：1。**原因：**策略槽报告可以说明文件加载情况；可执行skills单独清空，不能将“文件存在”与“本机能执行”混为一谈。

修改前：

```rust
                skills: state.policies.load().skills.clone(),
```

修改后：

```rust
                skills: if duck_control::morphology::current().headless() { Vec::new() } else { state.policies.load().skills.clone() },
```

# 新增Rust文件全文

以下也直接保存在payload对应目录中。

## `duck-control/src/morphology.rs`

```rust
//! Fixed physical topology without changing the 15-slot wire / 61 -> 14 policy ABI.
//!
//! Loaded once by robotd, before opening the bus. No hot reload, no discovery-based
//! masking: a configured leg that stops replying remains a real communication fault.

use std::path::Path;
use std::sync::OnceLock;

use serde::{Deserialize, Serialize};

use crate::io::Sensors;
use crate::model::{DEFAULT_POSITION, JOINT_IDS, JOINT_NAMES, MOUTH_INDEX, NUM_JOINTS};
use crate::obs::{BodyPose, Command};

pub const BUILD_ID: &str = "headless-v015-r1";
pub const CONFIG_ENV: &str = "MICRODUCK_MORPHOLOGY";
pub const HEAD_SLOTS: [usize; 4] = [5, 6, 7, 8];
pub const LEG_POLICY_SLOTS: [usize; 10] = [0, 1, 2, 3, 4, 9, 10, 11, 12, 13];

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Profile {
    Full15,
    Headless,
}

fn yes() -> bool { true }
fn home_head() -> [f64; 4] { HEAD_SLOTS.map(|j| DEFAULT_POSITION[j]) }
fn zero() -> f64 { 0.0 }
fn max_twist() -> [f64; 3] { [0.10, 0.04, 0.40] }

/// Only the four neck/head joints may be omitted as a group. The mouth is
/// independent. There is deliberately no arbitrary `ignore_ids` option.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Morphology {
    pub schema_version: u32,
    pub profile: Profile,
    #[serde(default = "yes")]
    pub mouth_present: bool,
    /// Absolute joint angles in the existing runtime coordinate system, radians.
    /// NOT offsets and NOT Dynamixel raw counts. HOME is subtracted by obs.rs.
    #[serde(default = "home_head")]
    pub locked_head_rad: [f64; 4],
    #[serde(default = "zero")]
    pub locked_mouth_rad: f64,
    /// Bench-first opt-in. Does not certify that a policy can balance this body.
    #[serde(default)]
    pub allow_motion: bool,
    #[serde(default = "max_twist")]
    pub max_twist: [f64; 3],
    /// Identical payloads alone do not prove staleness (quantization at rest).
    /// Keep the upstream diagnostic by default. Opt into a stop only AFTER
    /// validating this threshold against the actual IMU firmware on the bench.
    #[serde(default)]
    pub stop_on_identical_imu_blocks: Option<u64>,
}

impl Default for Morphology {
    fn default() -> Self {
        Self {
            schema_version: 1,
            profile: Profile::Full15,
            mouth_present: true,
            locked_head_rad: home_head(),
            locked_mouth_rad: 0.0,
            allow_motion: true,
            max_twist: max_twist(),
            stop_on_identical_imu_blocks: None,
        }
    }
}

static ACTIVE: OnceLock<Morphology> = OnceLock::new();

pub fn current() -> &'static Morphology {
    ACTIVE.get_or_init(Morphology::default)
}

/// No environment variable means the unmodified full-robot behavior.
/// A specified missing/invalid file is fatal, never a fallback to Full15.
pub fn initialize_from_env() -> Result<(), String> {
    let cfg = match std::env::var_os(CONFIG_ENV) {
        Some(path) if !path.is_empty() => Morphology::load(Path::new(&path))?,
        Some(_) => return Err(format!("{CONFIG_ENV} must not be empty")),
        None => Morphology::default(),
    };
    cfg.validate()?;
    ACTIVE.set(cfg).map_err(|_| "morphology initialized more than once".to_owned())
}

impl Morphology {
    pub fn load(path: &Path) -> Result<Self, String> {
        let bytes = std::fs::read(path).map_err(|e| format!("{}: {e}", path.display()))?;
        if bytes.len() > 65_536 { return Err("morphology file exceeds 64 KiB".to_owned()); }
        let cfg: Self = serde_json::from_slice(&bytes)
            .map_err(|e| format!("{}: {e}", path.display()))?;
        cfg.validate()?;
        Ok(cfg)
    }

    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != 1 { return Err("schema_version must be 1".to_owned()); }
        if self.profile == Profile::Full15 && !self.allow_motion {
            return Err("explicit full15 requires allow_motion=true; refusing a misleading disabled flag".to_owned());
        }
        if self.profile == Profile::Full15 && !self.mouth_present {
            return Err("full15 requires mouth_present=true; use headless for this patch".to_owned());
        }
        // Conservative validation envelope for this patch, not a mechanical certification.
        let lo = [-1.57, -1.57, -1.57, -0.35];
        let hi = [ 1.57,  1.57,  1.57,  0.35];
        for (i, &q) in self.locked_head_rad.iter().enumerate() {
            if !q.is_finite() || q < lo[i] || q > hi[i] {
                return Err(format!("locked_head_rad[{i}] is outside the permitted fixed-pose envelope"));
            }
        }
        if !self.locked_mouth_rad.is_finite()
            || !(-0.08..=0.52).contains(&self.locked_mouth_rad) {
            return Err("locked_mouth_rad must be finite in -0.08..=0.52".to_owned());
        }
        if self.stop_on_identical_imu_blocks.is_some_and(|n| n < 25) {
            return Err("stop_on_identical_imu_blocks must be null or at least 25".to_owned());
        }
        if self.max_twist.iter().any(|v| !v.is_finite() || *v <= 0.0) {
            return Err("max_twist must contain three positive finite limits".to_owned());
        }
        Ok(())
    }

    pub fn headless(&self) -> bool { self.profile == Profile::Headless }
    pub fn motion_allowed(&self) -> bool { !self.headless() || self.allow_motion }

    pub fn is_present(&self, joint: usize) -> bool {
        joint < NUM_JOINTS && (!self.headless()
            || (!HEAD_SLOTS.contains(&joint) && (joint != MOUTH_INDEX || self.mouth_present)))
    }

    pub fn active_slots(&self) -> impl Iterator<Item = usize> + '_ {
        (0..NUM_JOINTS).filter(|&j| self.is_present(j))
    }

    pub fn motor_ids(&self) -> Vec<u8> { self.active_slots().map(|j| JOINT_IDS[j]).collect() }

    pub fn is_absent_id(&self, id: u8) -> bool {
        JOINT_IDS.iter().position(|&x| x == id).is_some_and(|j| !self.is_present(j))
    }

    /// Fixed slots are configuration-derived, not sensor readings.
    pub fn project_positions(&self, positions: &mut [f64; NUM_JOINTS]) {
        if !self.headless() { return; }
        for (j, q) in HEAD_SLOTS.into_iter().zip(self.locked_head_rad) { positions[j] = q; }
        if !self.mouth_present { positions[MOUTH_INDEX] = self.locked_mouth_rad; }
    }

    pub fn seed_positions(&self) -> [f64; NUM_JOINTS] {
        let mut out = DEFAULT_POSITION;
        self.project_positions(&mut out);
        out
    }

    pub fn project_sensors(&self, sensors: &mut Sensors) {
        self.project_positions(&mut sensors.positions);
        for j in 0..NUM_JOINTS {
            if !self.is_present(j) {
                sensors.velocities[j] = 0.0;
                sensors.currents_ma[j] = 0.0;
            }
        }
    }

    /// Called AFTER skill arbitration, BEFORE Observation::build. Raw previous
    /// action is intentionally untouched, including the four unexecuted values.
    pub fn project_command(&self, command: &mut Command) {
        if !self.headless() { return; }
        command.head = std::array::from_fn(|i| self.locked_head_rad[i] - DEFAULT_POSITION[HEAD_SLOTS[i]]);
        command.body = BodyPose::default();
        for (value, limit) in command.twist.iter_mut().zip(self.max_twist) {
            *value = if value.is_finite() { value.clamp(-limit, limit) } else { 0.0 };
        }
    }

    /// Allocation-free pack in EXACT physical-ID order. Only the prefix is sent.
    pub fn pack_positions(&self, positions: &[f64; NUM_JOINTS]) -> ([f64; NUM_JOINTS], usize) {
        let mut out = [0.0; NUM_JOINTS];
        let mut count = 0;
        for j in self.active_slots() { out[count] = positions[j]; count += 1; }
        (out, count)
    }

    /// Excludes missing slots rather than reporting virtual zero temperatures as
    /// real measurements and diluting the mean. Returns logical joint index.
    pub fn thermal_summary(&self, temps: &[f64; NUM_JOINTS]) -> (usize, f64, f64) {
        let mut hottest = 0;
        let mut maximum = f64::MIN;
        let mut sum = 0.0;
        let mut count = 0;
        for j in self.active_slots() {
            if temps[j] > maximum { maximum = temps[j]; hottest = j; }
            sum += temps[j]; count += 1;
        }
        (hottest, maximum, sum / count as f64)
    }

    pub fn report(&self) -> serde_json::Value {
        serde_json::json!({
            "patch": BUILD_ID,
            "upstream": "a9ec4b2079ef8ee7904014089c885bb07d57d63c",
            "config": self,
            "active_motor_ids": self.motor_ids(),
            "virtual_motor_ids": (0..NUM_JOINTS).filter(|&j| !self.is_present(j)).map(|j| JOINT_IDS[j]).collect::<Vec<_>>(),
            "present_mask": (0..NUM_JOINTS).map(|j| self.is_present(j)).collect::<Vec<_>>(),
            "joint_names": JOINT_NAMES,
            "policy_observation_width": 61,
            "policy_action_width": 14,
            "state_note": "Absent slots are synthetic: fixed position, zero velocity/current, no temperature sensor. Not measured healthy servos.",
            "kinematics_note": "Official geometry is retained; camera/ToF FK is not a calibrated model of a removed or relocated head.",
            "dynamics_validated": false
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::obs::{ACTION_LEN, OBS_LEN, Observation};

    fn headless(mouth: bool) -> Morphology {
        Morphology { profile: Profile::Headless, mouth_present: mouth, ..Default::default() }
    }

    #[test]
    fn full_robot_unchanged() {
        let m = Morphology::default();
        assert_eq!(m.motor_ids(), JOINT_IDS.to_vec());
        let mut q = std::array::from_fn(|j| j as f64);
        let old = q; m.project_positions(&mut q); assert_eq!(old, q);
        assert_eq!(m.pack_positions(&q), (q, NUM_JOINTS));
    }
    #[test]
    fn headless_with_mouth_has_eleven_devices() {
        assert_eq!(headless(true).motor_ids(), vec![20,21,22,23,24,34,10,11,12,13,14]);
    }
    #[test]
    fn legs_only_has_ten_devices() {
        assert_eq!(headless(false).motor_ids(), vec![20,21,22,23,24,10,11,12,13,14]);
    }
    #[test]
    fn no_leg_can_be_ignored() {
        for mouth in [true, false] {
            let m = headless(mouth);
            for id in [10,11,12,13,14,20,21,22,23,24] { assert!(!m.is_absent_id(id)); }
            for id in [30,31,32,33] { assert!(m.is_absent_id(id)); }
            assert!(!m.is_absent_id(200));
            assert!(!m.is_present(15));
        }
    }
    #[test]
    fn mask_keeps_right_leg_indexes_after_missing_middle_slots() {
        let m = headless(false);
        let q = std::array::from_fn(|j| j as f64 + 0.25);
        let (packed, n) = m.pack_positions(&q);
        assert_eq!(&packed[..n], &[0.25,1.25,2.25,3.25,4.25,10.25,11.25,12.25,13.25,14.25]);
    }
    #[test]
    fn virtual_state_is_fixed_not_last_target() {
        let m = headless(false);
        let mut s = Sensors::default();
        s.positions = [9.0; NUM_JOINTS]; s.velocities = [7.0; NUM_JOINTS];
        s.currents_ma = [42.0; NUM_JOINTS];
        m.project_sensors(&mut s);
        for j in HEAD_SLOTS { assert_eq!(s.positions[j], DEFAULT_POSITION[j]); assert_eq!(s.velocities[j],0.0); assert_eq!(s.currents_ma[j],0.0); }
        assert_eq!(s.positions[10],9.0); assert_eq!(s.currents_ma[10],42.0);
    }
    #[test]
    fn home_subtraction_is_done_exactly_once_and_history_is_raw() {
        let m = headless(false);
        let mut s = Sensors::default(); s.positions = DEFAULT_POSITION;
        m.project_sensors(&mut s);
        let mut c = Command::default(); m.project_command(&mut c);
        let history = std::array::from_fn::<_, ACTION_LEN, _>(|j| j as f32 * 0.05);
        let o = Observation::build(&s.imu,&s.positions,&s.velocities,&DEFAULT_POSITION,&history,&c);
        assert_eq!(o.as_slice().len(),OBS_LEN);
        assert_eq!(&o.as_slice()[11..15], &[0.0;4]);
        assert_eq!(&o.as_slice()[34..48], &history);
    }
    #[test]
    fn non_home_lock_generates_measured_offset_and_command_offset() {
        let mut m = headless(true); m.locked_head_rad=[0.0;4];
        let mut s=Sensors::default(); s.positions=DEFAULT_POSITION; m.project_sensors(&mut s);
        let mut c=Command::default(); m.project_command(&mut c);
        let o=Observation::build(&s.imu,&s.positions,&s.velocities,&DEFAULT_POSITION,&[0.0;ACTION_LEN],&c);
        assert!((o.as_slice()[11] + 0.3491).abs() < 1e-6);
        assert!((o.as_slice()[51] + 0.3491).abs() < 1e-6);
    }
    #[test]
    fn discarded_output_is_not_taken_from_first_ten() {
        let a=std::array::from_fn::<_, ACTION_LEN, _>(|j| j as f32);
        let logical=Observation::scatter_action(&a);
        let (packed,n)=headless(false).pack_positions(&logical);
        assert_eq!(&packed[..n], &[0.0,1.0,2.0,3.0,4.0,9.0,10.0,11.0,12.0,13.0]);
    }
    #[test]
    fn physical_temperature_mean_not_diluted_by_virtual_zeroes() {
        let m=headless(false); let mut t=[0.0;NUM_JOINTS];
        for j in m.active_slots(){ t[j]=40.0; } t[12]=50.0;
        assert_eq!(m.thermal_summary(&t),(12,50.0,41.0));
    }
    #[test]
    fn bad_config_rejected() {
        assert!(serde_json::from_str::<Morphology>(r#"{"schema_version":1,"profile":"headless","ignore_ids":[13]}"#).is_err());
        assert!(serde_json::from_str::<Morphology>(r#"{}"#).is_err());
        let mut m=headless(false); m.schema_version=2; assert!(m.validate().is_err());
        m.schema_version=1; m.locked_head_rad[0]=f64::NAN; assert!(m.validate().is_err());
        m.locked_head_rad=home_head(); m.max_twist[1]=0.0; assert!(m.validate().is_err());
    }
    #[test]
    fn configured_headless_is_bench_only_without_opt_in() {
        let m: Morphology=serde_json::from_str(r#"{"schema_version":1,"profile":"headless"}"#).unwrap();
        assert!(!m.motion_allowed()); assert!(m.mouth_present);
    }
    #[test]
    fn command_head_is_not_controllable_and_twist_is_limited() {
        let mut c=Command{twist:[9.0,-9.0,9.0],head:[1.0;4],body:BodyPose{z:0.03,roll:0.5,pitch:0.5}};
        headless(false).project_command(&mut c);
        assert_eq!(c.twist,[0.10,-0.04,0.40]); assert_eq!(c.head,[0.0;4]); assert_eq!(c.body,BodyPose::default());
    }
}

```

## `robotd/src/headless.rs`

```rust
//! Daemon-only policy/capability guard for the fixed-head prototype.
//! The public 0.15.0 protocol types and its shared TOML schema are unchanged.

use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use duck_control::io::{RobotIo, Sensors};
use duck_control::morphology;
use duck_control::safety::Safety;
use crate::{Intents, RobotState, proto};

// 0 = clear, 1 = fallen, 2 = frozen IMU, 3 = repeated bus failures.
static FAULT: AtomicU8 = AtomicU8::new(0);
static TORQUE_CUT: AtomicBool = AtomicBool::new(false);
static REARM: AtomicBool = AtomicBool::new(false);

pub fn fault_reason(code: u8) -> Option<&'static str> {
    match code {
        0 => None,
        1 => Some("headless fall latch: support and upright the robot, then explicitly rearm"),
        2 => Some("headless repeated-IMU-block latch: verify sample freshness and quantization before rearming"),
        _ => Some("headless bus fault latch: all configured physical servos must answer"),
    }
}

pub fn latched() -> bool { FAULT.load(Ordering::Acquire) != 0 }

fn cause(fallen: bool, stale: u64, errors: u64, limit: u64, stale_limit: Option<u64>) -> u8 {
    if errors >= limit { 3 }
    else if stale_limit.is_some_and(|n| stale >= n) { 2 }
    else if fallen { 1 } else { 0 }
}

fn upright_and_still(s: Option<&Sensors>) -> bool {
    s.is_some_and(|s| {
        s.imu.gravity.iter().all(|x| x.is_finite())
            && s.imu.gravity[2] < -0.94
            && s.imu.gyro.iter().all(|x| x.is_finite())
            && s.imu.gyro.iter().map(|x| x*x).sum::<f64>() < 0.09
    })
}

/// Run in the owner of the bus, before taking the command snapshot. Torque-off
/// is retried on failure, not silently treated as success. Rearming NEVER
/// enables torque or the policy; a separate explicit init/Start is necessary.
pub fn enforce_stop<T: RobotIo>(
    safety: &mut Safety<T>, state: &RobotState, intents: &Intents,
    fresh: Option<&Sensors>,
) -> bool {
    if !morphology::current().headless() { return false; }
    let error_count = state.consecutive_errors.load(Ordering::Relaxed);
    let why = cause(safety.imu_ready() && safety.fallen(), safety.imu_stale().run,
                    u64::from(error_count), u64::from(state.max_consecutive_errors.max(1)),
                    morphology::current().stop_on_identical_imu_blocks);
    if why != 0 && FAULT.compare_exchange(0, why, Ordering::AcqRel, Ordering::Acquire).is_ok() {
        tracing::error!(reason = fault_reason(why).unwrap_or("fault"), "headless motion latched off");
        TORQUE_CUT.store(false, Ordering::Release);
    }
    if REARM.swap(false, Ordering::AcqRel) {
        if why == 0 && error_count == 0 && safety.imu_ready()
            && !intents.enabled() && upright_and_still(fresh)
            && (!latched() || TORQUE_CUT.load(Ordering::Acquire)) {
            FAULT.store(0, Ordering::Release);
            TORQUE_CUT.store(false, Ordering::Release);
            tracing::warn!("headless latch cleared; torque stays OFF; init/Start is still required");
        } else {
            tracing::warn!("headless rearm refused: need fresh, upright, still sensors and acknowledged torque-off");
        }
    }
    if !latched() { return false; }
    intents.set_enabled(false);
    // Drop a pre-existing init request as well as refusing new ones at the IPC door.
    let _ = intents.take_power_request();
    if !TORQUE_CUT.load(Ordering::Acquire) {
        match safety.set_torque(false) {
            Ok(()) => TORQUE_CUT.store(true, Ordering::Release),
            Err(e) => tracing::warn!(error = %e, "headless torque-off incomplete; will retry"),
        }
    }
    true
}

/// Both JSON-RPC requests and notifications pass this gate. The controller and
/// final hardware writer independently enforce the fixed slots as defense in depth.
pub fn refusal(state: &RobotState, intents: &Intents, call: &proto::Call) -> Option<&'static str> {
    let m = morphology::current();
    if !m.headless() { return None; }
    match call {
        proto::Call::RobotHead(_) | proto::Call::RobotLook(_) =>
            Some("neck/head joints are not installed; fixed virtual pose is configured at startup"),
        proto::Call::RobotPose(_) => Some("body-pose mode is disabled for initial fixed-head validation"),
        proto::Call::RobotMouth(_) if !m.mouth_present => Some("mouth ID 34 is not installed"),
        proto::Call::RobotRebootMotors(p) if p.ids.iter().any(|id| !m.motor_ids().contains(id)) =>
            Some("reboot list contains an ID that is not a configured physical servo"),
        proto::Call::RobotDo(_) | proto::Call::RobotSetSkill(_) =>
            Some("whole-body skills are disabled on the fixed-head prototype"),
        proto::Call::RobotSetMode(_) => Some("drive-mode switching is disabled on the fixed-head prototype"),
        proto::Call::RobotChorale(p) if p.active => Some("chorale head motion is disabled on this hardware"),
        proto::Call::RobotInit => motion_refusal(state, m.motion_allowed()),
        proto::Call::RobotEnable(p) => {
            let on = if p.toggle { !intents.enabled() } else { p.on };
            if on { motion_refusal(state, m.motion_allowed()) } else { None }
        }
        _ => None,
    }
}

fn motion_refusal(state: &RobotState, allowed: bool) -> Option<&'static str> {
    if !allowed { return Some("bench-only profile: set allow_motion=true and restart after the protected checks"); }
    if let Some(reason) = fault_reason(FAULT.load(Ordering::Acquire)) { return Some(reason); }
    if state.fallen.load(Ordering::Relaxed) { return Some("upright the fixed-head robot before enabling motion"); }
    if !state.imu_ready.load(Ordering::Relaxed) { return Some("IMU is not ready"); }
    if state.consecutive_errors.load(Ordering::Relaxed) > 0 { return Some("motor bus is not currently healthy"); }
    None
}

/// Local Unix-socket extension; no new Call variant and no required rebuild of
/// robotctl, padd, btd or the shared Params consumers. Notifications do not rearm.
pub fn response(state: &RobotState, intents: &Intents, id: proto::Id, method: &str) -> proto::Response {
    if method == "robot.morphology.rearm" {
        let result = if !morphology::current().headless() {
            proto::IntentResult::refused("not a headless profile")
        } else if intents.enabled() {
            proto::IntentResult::refused("disable/relax first; rearming never enables motion")
        } else {
            REARM.store(true, Ordering::Release);
            // The control thread must decide against a fresh sensor sample.
            proto::IntentResult::accepted()
        };
        return proto::Response::ok(Some(id), &result);
    }
    let mut report = morphology::current().report();
    report["fault_latched"] = serde_json::json!(latched());
    report["fault_reason"] = serde_json::json!(fault_reason(FAULT.load(Ordering::Acquire)));
    report["torque_off_acknowledged"] = serde_json::json!(TORQUE_CUT.load(Ordering::Acquire));
    report["rearm_pending"] = serde_json::json!(REARM.load(Ordering::Acquire));
    report["policy_enabled"] = serde_json::json!(intents.enabled());
    report["imu_ready"] = serde_json::json!(state.imu_ready.load(Ordering::Relaxed));
    proto::Response::ok(Some(id), &report)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn stop_causes_are_explicit() {
        assert_eq!(cause(false, 24, 9, 10, Some(25)), 0);
        assert_eq!(cause(true, 0, 0, 10, None), 1);
        assert_eq!(cause(false, 25, 0, 10, Some(25)), 2);
        assert_eq!(cause(false, 0, 10, 10, None), 3);
        assert_eq!(cause(false, 1000, 0, 10, None), 0, "repeated values need not be stale");
    }
    #[test]
    fn rearm_needs_real_upright_still_sample() {
        assert!(!upright_and_still(None));
        let mut s = Sensors::default(); s.imu.gravity=[0.0,0.0,-1.0]; s.imu.gyro=[0.0;3];
        assert!(upright_and_still(Some(&s)));
        s.imu.gyro[0]=0.4; assert!(!upright_and_still(Some(&s)));
        s.imu.gyro=[0.0;3]; s.imu.gravity[2]=0.0; assert!(!upright_and_still(Some(&s)));
        s.imu.gravity[2]=f64::NAN; assert!(!upright_and_still(Some(&s)));
    }
}

```

## `robotd/tests/headless_profile.rs`

```rust
//! Process-isolated CLI/socket tests. No serial device, ONNX or physical robot.
#![cfg(unix)]
use std::fs;
use std::io::{BufRead, BufReader, Write};
use std::os::unix::net::UnixStream;
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};
use serde_json::{Value, json};

static NEXT: AtomicU64 = AtomicU64::new(0);
fn directory() -> PathBuf {
    // Short enough for macOS's Unix-socket path limit; create_dir refuses collisions.
    let p = PathBuf::from(format!("/tmp/md-hl-{}-{}", std::process::id(), NEXT.fetch_add(1, Ordering::Relaxed)));
    fs::create_dir(&p).unwrap(); p
}
struct Robot { child: Child, root: PathBuf }
impl Drop for Robot {
    fn drop(&mut self) {
        let _ = self.child.kill(); let _ = self.child.wait(); let _ = fs::remove_dir_all(&self.root);
    }
}
impl Robot {
    fn start(mouth: bool, full: bool) -> Self {
        let root = directory();
        fs::write(root.join("m.json"), serde_json::to_vec(&json!({
            "schema_version":1, "profile":if full {"full15"} else {"headless"},
            "mouth_present":mouth, "allow_motion":full
        })).unwrap()).unwrap();
        fs::write(root.join("robotd.toml"), "[policy]\nenabled=false\n[audio]\nenabled=false\n[safety]\nbattery_empty_shutdown=false\n").unwrap();
        let log = fs::File::create(root.join("log")).unwrap();
        let child = Command::new(env!("CARGO_BIN_EXE_robotd"))
            .args(["--fake", "--no-policy", "--socket"]).arg(root.join("s"))
            .arg("--params").arg(root.join("robotd.toml"))
            .env("MICRODUCK_MORPHOLOGY", root.join("m.json"))
            .env("DUCK_RUNTIME_DIR", root.join("runtime"))
            .stdout(Stdio::null()).stderr(log).spawn().unwrap();
        let mut robot = Self {child, root};
        let until = Instant::now()+Duration::from_secs(10);
        loop {
            if robot.child.try_wait().unwrap().is_some() {
                panic!("daemon exited: {}",fs::read_to_string(robot.root.join("log")).unwrap());
            }
            if UnixStream::connect(robot.root.join("s")).is_ok() {break;}
            assert!(Instant::now()<until,"daemon socket did not appear");
            std::thread::sleep(Duration::from_millis(20));
        }
        robot
    }
    fn call(&self, method: &str, params: Value) -> Value {
        let mut s = UnixStream::connect(self.root.join("s")).unwrap();
        s.set_read_timeout(Some(Duration::from_secs(3))).unwrap();
        s.set_write_timeout(Some(Duration::from_secs(3))).unwrap();
        let mut msg=serde_json::to_vec(&json!({"jsonrpc":"2.0","id":1,"method":method,"params":params})).unwrap();
        msg.push(b'\n'); s.write_all(&msg).unwrap();
        let mut line=String::new(); BufReader::new(s).read_line(&mut line).unwrap();
        serde_json::from_str(&line).unwrap()
    }
}
#[test]
fn both_missing_head_profiles_keep_policy_contract_and_refuse_unsafe_requests() {
    for mouth in [true,false] {
        let r=Robot::start(mouth,false);
        let report=r.call("robot.morphology",json!({})); let m=&report["result"];
        let ids=if mouth {json!([20,21,22,23,24,34,10,11,12,13,14])} else {json!([20,21,22,23,24,10,11,12,13,14])};
        assert_eq!(m["active_motor_ids"],ids); assert_eq!(m["policy_observation_width"],61); assert_eq!(m["policy_action_width"],14);
        for (method,p) in [
            ("robot.head",json!({"neck_pitch":0.1,"head_pitch":0.1,"head_yaw":0.1,"head_roll":0.1})),
            ("robot.do",json!({"skill":"roulade"})),
            ("robot.enable",json!({"on":true})),
            ("robot.init",json!({})),
            ("robot.rebootMotors",json!({"ids":[30]})),
        ] { let out=r.call(method,p); assert_eq!(out["result"]["accepted"],false,"{method}: {out}"); }
        let out=r.call("robot.mouth",json!({"open":0.5})); assert_eq!(out["result"]["accepted"],mouth);
        let skills=r.call("robot.skills",json!({}));assert_eq!(skills["result"]["skills"],json!([]));assert_eq!(skills["result"]["built_in"],json!([]));
        let sub=r.call("robot.subscribe",json!({"hz":5}));assert!(sub["result"]["sitstand"].is_null());assert!(sub["result"]["ground_pick"].is_null());assert_eq!(sub["result"]["skills"],json!([]));
    }
}
#[test]
fn explicit_full15_keeps_existing_head_intent() {
    let r=Robot::start(true,true); let m=r.call("robot.morphology",json!({}));
    assert_eq!(m["result"]["active_motor_ids"].as_array().unwrap().len(),15);
    let out=r.call("robot.head",json!({"neck_pitch":0.1,"head_pitch":0.1,"head_yaw":0.1,"head_roll":0.1}));
    assert_eq!(out["result"]["accepted"],true);
}
#[test]
fn specified_missing_config_is_fatal_before_bus_open() {
    let root=directory();
    let out=Command::new(env!("CARGO_BIN_EXE_robotd"))
        .args(["--fake","--no-policy","--socket"]).arg(root.join("s"))
        .env("MICRODUCK_MORPHOLOGY",root.join("missing.json"))
        .env("DUCK_RUNTIME_DIR",root.join("runtime"))
        .output().unwrap();
    assert!(!out.status.success());assert!(!root.join("s").exists());
    assert!(String::from_utf8_lossy(&out.stderr).contains("bad morphology"));
    fs::remove_dir_all(root).unwrap();
}

```
