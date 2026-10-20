# IMU保护：一条命令完成一组实验

本流程使用 `0.16.3-imu16p4-quality-63e0fa0c` 候选，默认保护窗口40ms。
它是**有支撑的实验候选，不是已通过实机验收的正式版本**。
保护机制、故障后保持最后目标等限制见[robotd设计](../design/robotd-design.md)。
本次工具没有在实机运行；本地测试通过不等于鸭子已经站稳。

## 只按这个顺序做

每组单独运行；脚本不会自动进入下一组。命令均在Mac执行，不用另开SSH终端。
准备时关闭所有控制网页。脚本会停止 `padd/btd/mediad`，结束后保持停止。
`init` 是到 **HOME**，不是 seat。

### 1. 支架无动力故障响应检查

把**躯干**固定在支架上，近似竖直；双脚悬空，支架、线缆和手都不碰腿脚。
不能仅靠舵机保持机身。保持固定，全程不要调整或倾斜。

```bash
cd /Users/bou/Workspace/Duckduck
bash tools/zero3w_imu_accept.sh --stage bench --long
```

看到准备提示后输入 `YES`。首次会安装保护候选并重启robotd，随后确认relax；
已安装同一候选则复用。**本组不会执行init或enable，不会向真实传感器注入假数据。**
自动标记 `bench_fixed_static_begin/end`，静止采集180秒，最后按提示确认实物是否一直静止。
自动封存原始payload、状态、日志、配置/模型身份、标签和SHA256，并取回Mac。

结果看终端和 `result.json`：

| 结果 | 含义与下一步 |
|---|---|
| `PASS` | 捕获至少约1秒的真实unknown锁定；无效姿态始终不可用于控制、预测未超过40ms，保持relax、无自动恢复、记录完整，人工确认未动作。可以进行下一组**受保护的短时承重实验**。 |
| `NOT_EXERCISED` | 没有充分触发unknown；仅完成静态采集，**不能说故障响应通过**。保持支架，重新运行同一命令；不准靠倾斜、拔线或碰撞制造故障。 |
| `FAIL` | 数据/身份/保护/实物检查不满足条件；保持relax，先分析日志，不进入下一组。 |

本组验证的是无动力保护链路。目标/增益/使能写入被拒绝的证明来自Rust测试；
这份日志没有独立总线嗅探，不能冒充实际总线写入计数或承重故障测试。
如果多次没有自然故障，下一步应做隔离总线台架注入/嗅探；不要把等待无故障改成PASS。

### 2. 零指令承重站立

只有本机上一组 `PASS` 才会放行；程序、参数、模型、主控身份变化或重启服务会拒绝。
换HAT/IMU等脚本不能识别的硬件，也必须重新从第1组开始。
发出动力指令前，还会要求当前姿态有效、非预测且推导倾角≤15°，
与下面要求的近似竖直摆放一致。15°只是实验起始摆放检查，不改变生产异常检测阈值。
超过范围就保持relax分析；不能通过重置或修改校正值硬绕过。

把鸭子放在防滑平地，双脚真正承重，腿脚完全不接触支架。
托住躯干，使用松弛的防倒保护绳/能及时托住的安排；**全程有人照看**。
保护锁定会停止更新目标，不能保证继续平衡；异常时或5分钟板端兜底超时会relax。

```bash
cd /Users/bou/Workspace/Duckduck
bash tools/zero3w_imu_accept.sh --stage stand
```

按终端提示：

1. 输入 `YES`，脚本relax、核对上一组及身份。
2. 托稳后输入 `YES`，脚本执行init到HOME、等4秒。
3. HOME完成且没卡住，输入 `YES`，脚本enable、等3秒。
4. 确认双脚承重且已经能站稳，输入 `YES` 后撤手；保护绳保持松弛。
5. 观察30秒。结束后先托稳，再回答末尾提示；脚本relax并取日志。

脚本自动打 `init_home`、`enable`、`hands_off_zero_stand_begin/end` 标签。
末尾的 `YES` 同时确认：没有持续抖动、自行后退、倒地、扶正或支架施力；
如果有任一项，回答 `NO`。脚本不能凭零速度指令证明实物没有移动。

**只有 `PASS` 才进入第3组。**要求30秒全段零请求/应用速度、策略正常运行、
无unknown/倒地/拾起/relax、记录完整，以及人工实物检查通过。
即使显示PASS，也只证明这一小段，不能证明偶发故障已根除。

### 3. 短距离行走

保持同一地面与配置，清空前方至少0.5米，有人随行保护。上一组PASS才放行。

```bash
cd /Users/bou/Workspace/Duckduck
bash tools/zero3w_imu_accept.sh --stage walk
```

按相同提示完成HOME、enable、30秒零指令站立。
收到前进提示并输入 `YES` 后，脚本发送 **0.05m/s、3秒** 的前进指令，
随后stop并观察5秒；不是保证实物精确走15厘米。最后托稳，确认实物结果，relax。
自动打 `walk_forward_begin/end`、`walk_stopped_begin/end` 标签。
不测试后退、转弯、翻滚或摔倒起身。

## 发生异常时只做两件事

- 尚可安全观察：在正在运行的终端输入 `b` 回车，标记 `backward_drift`。本轮不会判PASS。
- 需要停止：**先托稳躯干**，输入 `s` 回车，自动记录 `operator_supported_stop`、
  stop、relax、封存取回。若通信断开，不要等脚本来托住鸭子；继续保持物理支撑。

提示确认有30秒期限，超时终止。SSH断开会尝试清理；板端另有5分钟relax定时器，
它不是瞬时急停。前进指令还有robotd原有deadman，停止前进不等于释放扭矩。
只有收到新 `robot.relax: torque off` 日志才算清理成功。
脚本只标注操作发生时间，不假称比人工反应更早捕获“真实后退开始”。

## 日志与恢复

正常结束自动取回到：

```text
/Users/bou/Workspace/Duckduck/logs/imu-acceptance/<组别-时间-编号>/<组别-时间-编号>/
```

优先看 `result.json`。`imu-payload-full.jsonl` 保留准备/HOME/enable等全程原始数据；
`imu-payload.jsonl` 是本组判读窗口的子集；
`events.jsonl` 对齐动作标签；`state.jsonl` 有控制指令、目标和实测关节；
`commands.jsonl` 记录脚本发送的每条请求、时间和应答；
`journal.log` 保存状态切换，`snapshot.json` / `end/snapshot.json` 核对身份。
缺日志、尾部缺失、丢记录、错误模式或中途换程序都不能算PASS。
所有日志保留在板端，Mac断网后用同一入口重取：

```bash
bash tools/zero3w_imu_accept.sh --fetch-last
```

目标IP改变时在命令末尾加 `root@新IP`，三组须针对同一只鸭子。
成功测试结束**不自动rollback**，保留候选且relax；安装验证失败会尝试恢复原服务。
要回到安装前版本，先固定躯干，再运行：

```bash
bash tools/zero3w_imu_accept.sh --rollback
```

## 其他checkout如何使用

Mac根目录入口是上述本地工作区适配；仓库里的正式入口为
`scripts/imu-acceptance/run.sh`。先准备SHA固定的已审计二进制包（仓库不提交二进制）：

```bash
python3 scripts/imu-acceptance/package.py /path/to/offline-audited-package /path/to/bench-package
export IMU_ACCEPT_PACKAGE=/path/to/bench-package
export IMU_ACCEPT_OUTPUT=/path/to/logs
bash scripts/imu-acceptance/run.sh --check-local
bash scripts/imu-acceptance/run.sh --stage bench --long root@192.168.1.56
```

打包器只接受这次已验证的二进制及源码SHA，不把任意构建伪装为已验收版本。
修改控制源码后，需要重新完整构建、回放及更新明确的候选身份。

## 本地验证证据与限制

这次控制源码SHA为 `63e0fa0c4d48555109df5c9b4286908e92bcd8f112e375bb0447f8c7518872a1`；
ARM64二进制SHA为 `d19c9ccf586c27a0b1581cab35fcad57fb9c7746e014953fb38429201d919a39`。
当前源码重新运行336项常规Rust测试通过，一键工具17项Python测试通过。
同一控制源码此前18项ONNX测试、ARM64构建和Linux严格Clippy通过。

完整日志重放95,822条不同时间/payload记录，另有7,566条重叠快照，不重复计作独立证据：
A组22,415条正常；B组22,385条中1帧预测替代且恢复；15,087条持续半值日志
在40.298ms后锁定；35,935条长日志在持续倾斜不一致40.019ms后锁定。
保护验收通过，**连续运行/实物稳定性仍未验收**。1000ms仅供离线对比，不用于此脚本。
锁定后不恢复控制及总线入口拒写有软件回归；不能据此说真实舵机已停止物理运动。

拿到原始封存日志后，可用仓库工具重现完整重放：

```bash
cargo build --release -p duck-control --example imu-trace-replay
python3 scripts/imu-acceptance/replay_full.py \
  --data-root /Users/bou/Workspace/Duckduck \
  --output /Users/bou/Workspace/Duckduck/logs/imu-replay-pr \
  --executable target/release/examples/imu-trace-replay
python3 -m unittest discover -s scripts/imu-acceptance -p 'test_*.py' -v
```

源日志SHA、条数和逐行输入不变性在工具中校验；缺少完整日志会失败，不能用摘取片段冒充。
