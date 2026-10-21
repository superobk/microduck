# IMU quality candidate: staged retest and recovery diagnosis

For the current candidate, use the [one-command supported acceptance guide](imu-acceptance.md).
The manual steps below describe the earlier trace/shadow investigation, not an acceptance pass.

This branch starts at official `daemon-v0.16.3` plus the separately committed
16.4-counts/(degree/s) gyro conversion. It is not an official signed release.
Shadow mode preserves that candidate's decoded outputs; predict mode is a bench
experiment. Mechanism and failure semantics belong to
[robotd-design](../design/robotd-design.md).

## 当前可做的两轮操作

以下针对已确认目标 `root@192.168.1.56`。不因本文自动安装新程序，也不自动执行动力。
先完成A轮，再完成B轮；每轮单独抓取。两轮保持原IMU、模型、增益不变。
`observe`只启动诊断，不启动/重启padd或控制服务；`mark`只记录时间，不停止机器人。
温度随健康数据记录，不增加启动温度门槛。

### A轮：停手柄，仅测试零指令站立

1. 在Mac开两个终端。终端1连接板端：

   ```bash
   ssh root@192.168.1.56
   systemctl is-active robotd
   robotctl policy list --json
   ```

   应为active，walk/sitstand/ground_pick/roulade槽没有加载错误。
   这些命令是只读；不能仅凭模型文件存在就判定正在运行。

2. 实际托稳躯干，终端1执行：

   ```bash
   robotctl robot relax --yes --json
   journalctl -u robotd -b -n 12 --no-pager
   systemctl stop padd
   systemctl is-active padd
   ```

   核对`robot.relax: torque off`，padd应为inactive；is-active此时非零退出码正常。
   关闭手机/网页等其他运动控制入口，手柄不按动作键。

3. Mac终端2，在当前Duckduck工作区执行：

   ```bash
   cd /Users/bou/Workspace/Duckduck
   bash tools/zero3w_reboot.sh observe 180
   bash tools/zero3w_reboot.sh mark A_pad_isolated
   ```

4. 终端1、继续托稳，单独执行init：

   ```bash
   robotctl robot init --json
   journalctl -u robotd -b -n 15 --no-pager
   ```

   等init姿态过渡完成；accepted是收件确认，不等于动作完成。
   若出现异常，托稳并relax，直接跳到第8步。
   完成后终端2标记：

   ```bash
   bash tools/zero3w_reboot.sh mark A_init_done
   ```

5. 终端1再单独执行：

   ```bash
   robotctl robot enable --json
   ```

   确认实际进入rise/walk，再在终端2标记：

   ```bash
   bash tools/zero3w_reboot.sh mark A_enable_done
   bash tools/zero3w_reboot.sh mark A_feet_loaded_supported
   ```

6. 双脚在地面承重，轻扶躯干20秒。若稳定，标记后短时撤去接触、双手留在旁边可立即扶住：

   ```bash
   bash tools/zero3w_reboot.sh mark A_hands_off_begin
   ```

   只观察5–10秒，不操作行走/翻滚。若不适合撤手，保留轻扶，标记
   `A_hands_off_skipped`，不要假装已完成无人接触检查。

7. 一旦后退/明显摆动，先手动托稳，终端1立即执行：

   ```bash
   robotctl robot relax --yes --json
   ```

   然后终端2补记事实：

   ```bash
   bash tools/zero3w_reboot.sh mark A_backward_observed_late
   bash tools/zero3w_reboot.sh mark A_manual_support_late
   bash tools/zero3w_reboot.sh mark A_relax_done
   ```

   这些late标签明确晚于实际动作。若正常结束，则先托稳relax，只记
   `A_test_end`和`A_relax_done`，不打“后退”标签。

8. 终端2结束并取回本轮：

   ```bash
   bash tools/zero3w_reboot.sh stop
   bash tools/zero3w_reboot.sh fetch
   ```

   若180秒已经结束，不能往已结束文件补动作标签；先fetch，再新开一轮。
   collector停止不会替你释放扭矩。

### B轮：始终relax，单独记录手柄静置和回中

1. 机器人托架放稳，终端1再次relax并核对日志，然后只启动手柄服务：

   ```bash
   robotctl robot relax --yes --json
   systemctl start padd
   robotctl pad status --json
   robotctl pad bindings --json
   ```

   确认手柄connected。全轮不按Start/Select、A/B/X/LB/RB等动作键，
   不执行init/enable。仅relax属于软件释放扭矩，不是物理切断舵机电源。

2. 终端2：

   ```bash
   bash tools/zero3w_reboot.sh observe 120
   bash tools/zero3w_reboot.sh mark B_pad_untouched_begin
   sleep 20
   bash tools/zero3w_reboot.sh mark B_pad_untouched_end
   ```

   这20秒手柄放桌上，双手不碰摇杆。

3. 标记后，慢推左摇杆前后各一次，再完全松开；此段输入是刻意测试：

   ```bash
   bash tools/zero3w_reboot.sh mark B_stick_cycle_begin
   ```

   完成并释放后：

   ```bash
   bash tools/zero3w_reboot.sh mark B_stick_released
   sleep 20
   bash tools/zero3w_reboot.sh mark B_center_wait_end
   ```

4. 标记后，只拿起/移动手柄，不碰摇杆和按钮，约10秒后放回：

   ```bash
   bash tools/zero3w_reboot.sh mark B_pad_move_without_stick_begin
   ```

   完成后：

   ```bash
   bash tools/zero3w_reboot.sh mark B_pad_move_without_stick_end
   ```

5. 终端1关闭手柄输入并确认仍relax：

   ```bash
   systemctl stop padd
   robotctl robot relax --yes --json
   ```

   终端2：

   ```bash
   bash tools/zero3w_reboot.sh mark B_relax_done
   bash tools/zero3w_reboot.sh stop
   bash tools/zero3w_reboot.sh fetch
   ```

   分析静置/释放时的ABS_Y与requested/applied。小幅中心偏移才适合校中心/死区；
   65535之类满量程事件需核查手柄/驱动来源，不能用普通死区解决。

## PR安装后的原始IMU观察配置

上面A/B能在现有imu16p4上做，但旧程序没有本文的新raw trace功能。
只有核对实际正在运行本PR编译产物后，下面环境变量才有意义。

安装只使用独立候选路径，不覆盖官方或原imu16p4程序，不改模型路径。
保持托稳并relax后，再进行用户手动的软件安装/服务重启。先备份实际
`systemctl cat robotd`和drop-in，保留回滚路径。

给**已经安装本PR程序**的robotd增加观察配置：

```bash
install -d -m 0700 /var/lib/robot/diagnostics/imu-raw
install -d -m 0755 /etc/systemd/system/robotd.service.d
cat > /etc/systemd/system/robotd.service.d/92-imu-quality-observe.conf <<'EOF'
[Service]
Environment=ROBOT_IMU_MODE=shadow
Environment=ROBOT_IMU_TRACE_DIR=/var/lib/robot/diagnostics/imu-raw
EOF
systemctl daemon-reload
systemctl restart robotd
robotctl robot relax --yes --json
journalctl -u robotd -b -n 35 --no-pager
```

核对`IMU quality monitoring configured`为Shadow，以及`IMU raw capture opened`。
新文件`imu-<monotonic_ns>.jsonl`来自唯一总线读取者。启动只加载模型不等于
init/enable；先做无动力静止/慢转/快速扶正观察，再重复A轮。

Mac取回raw trace可单独执行：

```bash
mkdir -p /Users/bou/Workspace/Duckduck/logs/imu-quality-raw-20261016
scp 'root@192.168.1.56:/var/lib/robot/diagnostics/imu-raw/imu-*.jsonl' \
  /Users/bou/Workspace/Duckduck/logs/imu-quality-raw-20261016/
```

raw trace不包含MCU FIFO字节或芯片采样序号。没有这些证据时，不把正常扶正
误称为固件错误。shadow标记仅供分析，控制输入保持原样。

`ROBOT_IMU_MODE=predict`不属于本轮自由站立指令。需先核对原始日志、在托架
测试真实翻转/故障/relax恢复，再考虑启用。它会改变控制输入，并在持续异常时
锁止策略；不能把它作为“已验证不会摔倒”的修复。

## 跌倒后翻滚/起身为什么不动或中途结束

- `roulade`是单次翻滚，当前默认约1秒；按住X的续接依赖150ms内重复请求。
  CLI的一次`robot.do`只启动一次，不会无限循环。按钮释放、输入服务停止或链路
  没有续请求时，窗口正常结束不等于模型加载失败。
- `ground_pick`是另一套地面起身模型，默认周期4秒、截止相位0.7，约2.8秒有效阶段。
  roulade结束不会自动调用它；落地后的`limp_pose→walk`也不表示ground_pick已执行。
- 坐姿、起立过渡、另一技能正在运行时，控制器会拒绝新技能。原版roulade等拒绝
  是debug日志，默认journal不容易看到；本PR首条拒绝升为warn，同一原因每秒最多一条。
- 正在执行的技能`busy()`会排除普通limp-fall和pickup观察，不能一概归因于
  “跌倒保护打断翻滚”。原始IMU/读取中断、策略disabled、relax/reboot、无模型和
  执行器带载跟踪失败仍须分别核对。
- 最近随机测试日志模型已加载，但未见`skill started`或`policy=ground_pick`；
  能确认没有捕捉到地面起身技能开始，不能证明某次已经开始的翻身因IMU中断。

单独诊断时，在另一SSH终端先开日志：

```bash
journalctl -fu robotd --no-pager
```

模型和输入映射只读检查：

```bash
robotctl policy list --json
robotctl pad bindings --json
```

在已由用户确认可做动作、policy仍enabled、机身有保护且周围无障碍时，
**二选一手动测试，不同时发送**：

```bash
robotctl robot do ground_pick --json
```

或单次翻滚：

```bash
robotctl robot do roulade --json
```

这两条都产生动力，不属于B轮relax手柄测试。若已relax，不要直接照抄技能命令
期待它运动；需另行托稳、分步init/enable。accepted只代表请求入队，要看到
`skill started`和实际policy标签，refused则按原因处理。
动作异常先托稳relax，再标`skill_stopped_observed_late`、fetch。不要通过延长
roll时间或自动接起身掩盖未定位故障；本PR没有更改这些动作时序。
