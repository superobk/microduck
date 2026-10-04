# 验证报告与审查门槛

RUN_ID：`zero3w-11servo-20261004T014630+0800`。完整日志位于工作区 `logs/<RUN_ID>`；每次命令在启动前记录argv/意义，结束后记录UTC、退出码及输出哈希，失败日志保留。候选源码与ARM产物身份见 `evidence/candidate-manifest.json`（完成候选构建后生成）。

| 验证 | 结果 | 证据/限制 |
|---|---|---|
| 基线哈希/锚点、原包24哈希 | 通过 | 原来源保护不放宽；原zippack未改 |
| Python应用器/配置/普通协议工具 | 30/30通过 | 包含.DS_Store、未声明文件与symlink负例 |
| 离线日志评估 | 17/17通过 | 不构成物理运动证据 |
| 审计/迁移输入保护与回退 | 11/11通过 | 失败记录/增量摘要、注入/供电门禁、plan-only；8种中断组合、重复回退、后续完整恢复、防漂移/symlink/伪造清单 |
| duck-control + robotd | 通过 | 92 core、171 daemon、4 profile、9单实例、7 updater gate；各日志有输出 |
| 工作区回归 | 1476通过；18原忽略项；1环境负例另列 | 最终重跑汇总以evidence日志索引为准，不能称完全无排除通过 |
| 真实ONNX Runtime策略记忆/重置 | 8/8通过 | Runtime1.23.2；原ignored测试显式执行 |
| 真实Runtime控制器反馈重置 | 1/1通过 | Runtime1.23.2；原ignored测试显式执行 |
| 实际Rust推理fake进程 | 有效参考模型、坏状态加载拒绝、运行期NaN锁止/关闭/显式复位 | 隔离socket/runtime，只验证软件路径 |
| ARM Linux/glibc2.31构建 | 通过 | Rust1.89、cargo-zigbuild/Zig0.13；最终提交嵌入revision重新构建 |
| Linux ARM PTY普通协议 | 10/11/15三组通过 | 实际候选binary；IMU/motor读顺序、按ID回填/打包 |
| Linux ARM PTY Fast Sync Read | 10/11/15三组通过 | 独立0x8A复合状态与runningCRC；不是从普通协议推断 |
| Linux候选fake与嵌入revision | 通过 | ARM容器执行与板端相同例程：ready、health、ABI37、revision；不是板端验收 |
| Linux PTY故障/复位 | 10/11 profile通过 | 掉线ID13、请求/通知门禁、ACK但OFF回读为ON、重试、拒绝/成功复位；15只回归原行为 |
| 本地9参考模型契约 | 9/9通过 | 文件哈希不变；无板端身份推定 |
| Rust/Python黄金向量 | 108帧通过 | 最大差5.55e-17 |
| 42次参考物理试验 | 基础3/9通过 | 詳見能力矩阵，不是硬件证据 |
| 板端只读预检 | 已执行 | 指定基线、有效配置、服务、模型路径、ORT两份C版本、ToF帧/相机统计；私有证据在新的inspect run |
| 板端安装/隔离fake | 待人工审计 | 本轮未远端写入或执行fake；不等于ARM容器的验收 |
| 带动力与实机动作 | 0 | 按授权暂缓；供电/机械/在场验收未完成 |

## 环境影响与先前失败

工作区原 `mediad::turn::tests::a_failure_names_its_cause_and_not_just_itself` 要求访问 `turn.invalid.` 得到DNS/resolve错误。本环境实际返回TLS handshake EOF；即使删除常见代理环境变量仍如此。该函数源码与指定基线相同；该负例不修改、不伪造通过，保留两次失败和其余回归结果。最终软件审查仍需在没有该网络拦截的环境补做无排除完整回归。

原工作区两个占位测试依赖ONNX Runtime不可加载，整套注入Runtime后遇到占位无效模型/预设错误原因冲突。因此通用工作区按原默认环境跑；真实 Runtime检查使用专用测试和隔离fake harness，二者结果分开记录。

原r1的PTY模拟把IMU字段当6个half并未正确表示gyro i16和安装旋转，新复位测试正确拒绝该高角速度/非直立样本。已修复模拟器，保持生产阈值及解码。先前失败输出和最终通过都保留，避免只展示最后绿色日志。

## 当前审查结论

本轮自审支持候选软件进入draft审查，未授权/未完成正式发布、合并或带动力部署。人的独立review仍待完成。局限包括板端运动模型缺失且实际加载Runtime尚未验证、未标定物理参数、移动工况失败、环境负例未补验。迁移工具只做已审查候选安装和隔离fake，存在真实板端差异时必须停止而非忽略。

## 2026-10-04 只读板端检查补充

RUN_ID 为 `zero3w-inspect-20261004T105654+0800`。原始输出和私有端点仅留在本地 logs，增量操作报告留在本地 docs/zero3w-operations。安装仍待人的独立审计。

- hello 与运行身份均为指定 a9ec4b2.../0.15.0/API37；原硬件 backend 非fake。root/AArch64，Python3.13.5、glibc2.41。
- 原 robotd 处于运行状态但 unhealthy，控制循环 ticks=0、IMU.ready=false、启动总线失败770次（第一份快照）。已有日志具体为 ID20 的 return_delay_time 读取超时，不能据此推断其他十个设备已坏或全部未应答。主控 IMU尚未开始有效读取，不是已证实IMU硬件损坏。
- tofd 现有流确认 VL53L5CX，8×8、配置15Hz，5个连续seq，收到时样本年龄0.61–0.76ms；仅证明短时数据流，距离准确度未标尺验证。head_imu 配置关闭，BMI088路径未实测。
- mediad 现有camera统计1280×720 UYVY、30fps；约9秒帧数增加270、dropped维持2。sysfs识别IMX219；尚无实际图像质量/WebRTC端到端验收。
- updaterd 运行/使能，check_interval=6h，auto_apply=mandatory；不能忽略其后续自动更新影响。
- ORT C API两份库分别1.21.0和1.28.0；robotd maps中尚无ORT加载。库存在及能返回版本并不是策略加载成功。运动模型路径事实由[模型能力文档](model-capabilities.zh-CN.md)维护。

本次有一次沙箱网络拒绝、一次SSH超时、一次journald MESSAGE为字节数组导致的辅助脚本异常，失败全部保留。字节数组解析修正后补查成功，不能抹掉先前失败记录。没有安装任何系统包，也没有更改远端文件、服务或设备配置。
