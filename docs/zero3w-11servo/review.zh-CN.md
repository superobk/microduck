# 改造审查：保持策略契约，改变物理设备集合

## 原材料结论

`zippack/10servos/microduck-headless-v015-r1` 是带锁定哈希和锚点的源码改造包。47 条规则、49 次替换涉及 4 个基线文件，另提供 morphology、daemon 门禁、进程隔离测试三个 Rust 文件；全部锚点匹配指定基线。24 个原包校验文件哈希匹配。其按 ID 收集/打包、缺失槽投影、能力拒绝、sidecar 配置、全 15 槽兼容设计合理，已复用。

原工具测试 27/28 的失败来自 `.DS_Store` 被目录遍历当作源码。当前导入器只接受明确列出的三个 Rust 路径，忽略 Finder 元数据，拒绝其他 payload、符号链接、异常基线及本地修改；新增负例后 30/30 通过。原包保持未修改。仓库 `tools/headless-v015-r1` 中保留的是导入来源/历史配方，`SHA256SUMS.original` 只验证原包；r2 以直接源码和 Git 提交为准，不能重新在 r2 上套用 r1。

`microduck-zero3w-rollout-r1` 是部署说明与离线日志分析补充，不含新固件或模型。复用了 `evaluate_log.py` 及 17 个测试；原部署文档中的操作边界被当前迁移指南替代。两份来源都没有原先的 Rust、ARM、真实 ONNX 和实机验证证明，当前新增的证据见 [验证报告](validation.zh-CN.md)。

## 模块职责与非显然改动

| 文件/模块 | 作用及修改理由 |
|---|---|
| `duck-control/src/morphology.rs` | 独立 sidecar 加载、物理掩码、按槽位投影、按 ID 打包、能力报告；缺失设备不能使右腿索引前移。默认完整构型；显式坏配置启动即失败。 |
| `duck-control/src/bus.rs` | 初始化、读、写、增益、扭矩、温度、重启都仅使用真实 ID；按原逻辑槽回填读取结果。头颈不在总线上，真实腿掉线仍报错。r2 关闭扭矩后逐 ID 回读 Torque Enable，防止把 ACK 当作已关闭。 |
| `robotd/src/control.rs` | 推理前仅投影缺失槽及命令；保留原 HOME、缩放、滤波、增益；检查输入转 float32 后和输出目标的有限值。模型原始 14 项动作继续进入历史反馈。 |
| `robotd/src/headless.rs` | 请求/通知共用能力拒绝；首个故障原因锁止；同 tick 关闭策略/清空待启用请求；关闭扭矩失败重试；复位由控制线程以新鲜数据决定。 |
| `robotd/src/main.rs` | 控制线程调用锁止；推理失败不再让修改构型继续依赖旧目标；health 暴露锁止。头颈固定构型禁用独立 CLI init，避免新进程绕过 daemon 的故障/感知门禁。坏策略加载时拒绝 init/enable。 |
| `robotd/tests/headless_profile.rs` | 独立进程验证 10/11/full15、错误配置及 CLI init 拒绝。各测试自己的 profile/runtime/socket，避免全局配置相互污染。 |
| `duck-control/examples/morphology-golden.rs` | 调用实际 Rust ABI、scatter、投影函数的离线黄金转换器。不给模型/总线提供替代实现。 |
| `tools/zero3w` | 独立仿真、真实 Runtime fake 验证、审计和分步迁移。假设备、参考模型和真实板端结果分别记录。 |

原 `model.rs`、`obs.rs`、`policy.rs`、共享 `duck-ipc-proto`、共享 `robotd-params` 和 Cargo.lock 保持基线内容。没有修改现有 ONNX。局部方法 `robot.morphology`、`robot.morphology.rearm` 通过本地 socket 扩展，不加入共享 Call/TOML 类型。全 15 构型仍使用原默认路径，启用能力与原版一致。

## 故障与复位的精确定义

故障包括跌倒、达到原连续总线失败阈值（默认10）、可选连续相同 IMU 块阈值、无效感知和推理失败。相同原始 IMU 字节可能是量化/静止，因此该可选阈值默认未启用；不能把相同字节直接宣称为物理传感器失效。推理有限值及新鲜度与姿态准备条件仍检查。

`valid_sensors` 检查真实关节位置/速度/电流及 IMU 有限值，重力与四元数平方范数位于 `[0.64,1.44]`。这是拒绝畸形样本的边界，尚未标定为实机误差模型。缺失头颈可投影，真实腿数据不可用零替代。

锁止后清除策略使能和排队 init，重试关闭所有真实舵机扭矩；每次包含寄存器回读。通信失败时软件无法证明动力已关闭，必须支撑并物理切断电机供电。日志说明失败；不把重试请求写作成功。

`robot.morphology.rearm` 返回 accepted 只代表排队。查询 `rearm_result=cleared` 且 `fault_latched=false` 才证明控制线程批准。批准要求本 tick 新鲜、有效、IMU ready、直立（gravity_z < -0.94）、静止（gyro范数 < 0.3 rad/s）、无连续总线错误、策略未启用、扭矩关闭已有完整回读；未满足则 refused。即使 cleared，动力和策略保持 OFF，后续仍需显式 init/enable。

锁止仅保存在本进程，重启不会保留。不能通过重启代替故障诊断或复位；重启只可在支撑且电机供电隔离时进行。当前服务保持停止的部署门禁属于本候选迁移机制，详见 [迁移指南](migration.zh-CN.md)。

## 自审结论与待审项

已检查右腿动作 9–13、缺失 ID 不在交易中、真实掉线锁止、请求/通知与独立 init 门禁、模型/ABI 不变、全15兼容；有对应测试输出。可进入候选软件审查。尚不满足正式发布/动力部署条件：板端运动模型与真实总线验收未完成（只读发现见[验证报告](validation.zh-CN.md)），参考模型移动工况未通过，存在一个环境相关工作区负例未通过，供电/机械未验收。PR 保持 draft，人的独立审查结果目前为待完成。
