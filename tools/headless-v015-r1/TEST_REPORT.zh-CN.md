# 测试报告：headless-v015-r1

## 已实际执行

| 检查 | 结果 | 证明范围 |
|---|---|---|
| `python -m unittest discover -s tests -v` | 28项通过 | Python工具代码、配置和协议编解码，不是Rust执行 |
| `python -m compileall -q .` | 通过 | Python语法 |
| `bash -n tools/verify.sh` | 通过 | Shell语法 |

原始Python测试输出：`test-results/tooling-unittest.log`。

覆盖：Git blob哈希与Git一致；缺失/重复锚点拒绝；顺序替换；47条规则的独立夹具；目标不涉及权重；路径越界/符号链接拒绝；文件写入保留模式；合成Git仓库中的幂等计划/反向恢复/用户改动拒绝；保留原ONNX路径、缩放、增益及低通；无末尾换行、空配置、TOML非法输入；5V选项、普通同步读取选项；禁止覆盖候选配置；两种JSON构型；标准DXL ping包CRC、字节填充、包长；模拟设备返回顺序、缺失设备不返回伪零、同步写入成员。

**合成Git仓库测试不能替代真实官方仓库应用检查。每个锚点的独立替换夹具，也不能证明47条规则在完整真实文件上全部匹配。**

## 已提供，但没有在本环境运行

| 层级 | 提供内容 | 当前状态 |
|---|---|---|
| 固定完整仓库应用 | `apply.py --check` 的commit/4个Git blob/全部锚点校验 | 未运行，当前没有完整checkout |
| Rust控制库测试 | morphology.rs内13项 | 未运行，无Rust/Cargo |
| Rust故障判断测试 | headless.rs内2项 | 未运行 |
| Rust进程集成测试 | headless_profile.rs内3项，含10/11构型 | 未运行 |
| 原仓库回归测试 | `cargo test --locked -p duck-control -p robotd` | 未运行 |
| 原生与AArch64构建 | `cargo build` / `cargo board` | 未运行 |
| 实际daemon伪串口 | `tools/pty_bus_test.py` 的两个构型场景 | 未运行，需要已构建Linux原生robotd |
| 原ONNX推理 | `tools/check_onnx.py`，10次CPU推理+SHA256 | 未运行，无ORT及用户权重 |
| 实际硬件 | 初始化、映射、IMU、温度、掉线、复位、站立/行走 | 未进行 |

## 本地验证顺序

先 `apply.py --check`，再应用及审阅代码，运行 `tools/verify.sh`。Linux上加 `--pty`。通过后构建ARM产物，再台架检查物理总线。最后核对原ONNX字节并进行保护性闭环试验。

遇到编译错误、锚点不匹配或PTY失败应停止部署，保存完整输出，不跳过检查。此交付中没有“已编译通过”的声明，也没有提供冒充实测的步行成功率。

## 已知限制

只支持四个头颈关节整体缺失，嘴独立可选，不支持任意屏蔽腿部。副配置不热加载。新诊断方法仅保证本地Unix socket入口。旧padd本地模式仍存在，需切回行走模式；官方相机/ToF模型未重新标定。台架模式不是纯只读，会保留原寄存器/保持目标操作。IMU重复值不等于采样停止，默认只诊断。故障锁止不跨重启，关闭扭矩可能使无支撑机体倒下。原权重能加载不等于动力学稳定。
