> 历史 r1 导入材料：这里的校验表仅适用于原始 zippack。当前可审查候选为仓库 Rust 源码的 r2；迁移仅按 [当前指南](../../docs/zero3w-11servo/migration.zh-CN.md) 执行。不要在 r2 上重跑 apply.py，也不要直接使用这里的旧 deploy 覆盖。

# Microduck 0.15.0：四个头颈舵机缺失时复用原 ONNX

交付版本：`headless-v015-r1`，2026-10-03。

**状态：源码实现及工具包，不是已验证固件。** 本包提供真实 Rust 模块、47项精确源码替换、配置文件、原生构建指引和测试代码。当前交付环境没有 Rust/Cargo、完整仓库容器副本、ONNX Runtime、用户权重或机器人，因此没有执行官方完整仓库上的应用检查、Rust编译、ARM构建、真实ONNX推理、串口联调或实机平衡测试。已经执行并通过的是28项Python工具测试、Python语法检查和Shell语法检查；详见 `TEST_REPORT.zh-CN.md`。必须先通过本机构建和接口测试，才进入机械保护下的硬件验收。

## 1. 实现目标与边界

唯一基线：`pollen-robotics/microduck` 的 `daemon-v0.15.0`，commit：

```text
a9ec4b2079ef8ee7904014089c885bb07d57d63c
```

这里的 `headless` 指“缺少头颈执行器”，不是图形界面的无显示器模式。

两种构型分别处理：

| 配置文件 | 不存在的DXL ID | 真实舵机数量 |
|---|---|---:|
| `profiles/headless-mouth.json` | 30、31、32、33；嘴34仍存在 | 11 |
| `profiles/legs10.json` | 30、31、32、33、34；嘴也不存在 | 10 |
| `profiles/full15.json` | 无 | 15 |

四个头颈自由度是 `neck_pitch / head_pitch / head_yaw / head_roll`。嘴是另外一个关节，不能把“缺4个”自动解释为“只剩10个”。此前只有双腿10舵机的硬件，应选 `legs10.json`。

**保留原始ONNX文件，不做图改写、重新导出、重新训练或10维网络替换。** 继续使用15个逻辑关节槽、61维观测、14维策略动作。只在真实设备边界减少总线成员，且把缺失头部的观测替换为配置约定的固定状态。

本方案不证明旧策略能够稳定控制新机械结构。固定头颈、拆除头部、改变电池位置或供电，都可能改变动力学。只有完成保护性试验后，才能评价你这台机器上的站立和行走表现。

### 硬件必须先满足

头颈结构如果仍在，必须用可靠刚性结构固定；不允许无舵机的关节自由摆动。固定角应对应官方关节坐标，而不是凭头壳相对地面角度推定。整个头部被移除时，虚拟角度仅是网络接口约定，不代表仍有对应质量或传感器。

双腿10个舵机必须全部正常，IMU ID200必须正常，原有舵机方向、归零、HOME和限位必须正确。本补丁不修复原有安装偏差，也不回移新版主线的膝关节标定逻辑。

改变构型、安装程序、恢复程序时，先机械支撑并切断舵机动力。给主控供电进行SSH操作，不等于允许电机动力上电。任何配置为缺失的电机都不再收到本程序的数据或扭矩指令，不能依靠本补丁把一台仍连接且已上扭矩的“被忽略电机”关掉。

## 2. 数据路径与最重要的索引

```text
真实IMU + 10/11个真实舵机
        ↓ active IDs → 原15逻辑槽 scatter
15槽状态：头颈=固定绝对角，头颈速度/电流=0
        ↓ 原 Observation::build，不改任何槽位
61维观测 → 原ONNX → 原始14维 action
        ├── 全14维保存到 last_action（包括未执行的头颈输出）
        ↓ 原HOME + 原缩放 + 原低通
15槽目标：头颈固定，缺失嘴部固定
        ↓ 原安全限位 + 物理成员 gather
只向10/11个真实舵机发送目标
```

| 部位 | 15槽索引 | DXL ID | 14策略动作索引 |
|---|---|---|---|
| 左腿 | 0..4 | 20..24 | 0..4 |
| 头颈 | 5..8 | 30..33 | 5..8 |
| 嘴 | 9 | 34 | 不参与策略 |
| 右腿 | 10..14 | 10..14 | 9..13 |

十舵机模式的真实总线顺序是：

```text
IMU+舵机读取：[200,20,21,22,23,24,10,11,12,13,14]
舵机写入：    [20,21,22,23,24,10,11,12,13,14]
```

保留嘴时，在左腿与右腿之间保留ID34。不能取 `action[:10]`，也不能将右腿读取结果继续紧接着写进逻辑槽5。

### 为什么固定头部不能随便填零

`locked_head_rad` 是**绝对关节角，单位弧度**。原 `obs.rs` 会自行减HOME。本包默认：

```json
"locked_head_rad": [0.3491, 0.3491, 0.0, 0.0]
```

这样两个俯仰关节的位置观测才是 `0.3491 - 0.3491 = 0`。配置为绝对0时，相对HOME的观测应为 `-0.3491`，不能再次补成零。头部命令也设置成 `locked - HOME`。这两个约20°是原关节坐标值，不等于把整个头壳相对地面倾斜20°。

### 为什么不清零头部 last_action

原合同明确 `last_action` 是上一帧**网络原始输出**，不是实际执行角或滤波目标。修改这一语义，会在删掉头部执行器之外，再引入一次观测分布变化。本包保留全14维历史，不虚构头部已经跟随动作移动。

真实IMU、双腿位置/速度/电流仍由实际传感器提供。仅配置中不存在的槽位被虚拟化，任何真实腿部读取失败仍走故障路径。

## 3. 为什么使用独立JSON，而不是往robotd.toml塞一个新字段

0.15.0的 `robotd-params` 使用严格未知字段拒绝，同一份TOML也由其他守护程序读取。只给robotd增加 `bus.ignore_ids`，但不升级其他消费者，可能造成它们无法加载同一个配置文件。

本包保持共享TOML类型、IPC枚举、模型常量和Cargo依赖不变。由robotd通过环境变量读取独立文件：

```text
MICRODUCK_MORPHOLOGY=/etc/robot/morphology.json
```

配置在打开总线前加载，一次启动只读一次。显式Full15文件必须包含 `allow_motion=true`，否则启动拒绝，避免配置显示禁止动作却实际回到完整机默认行为。指定了不存在、格式错误或未知字段的文件，会报错退出，**不会退回15舵机模式**。未指定该环境变量时，才保持完整15舵机默认行为。

新增本地Unix socket诊断方法 `robot.morphology` 和 `robot.morphology.rearm`。它们不经过新IPC枚举，不要求旧 `robotctl/padd/btd` 重编译；旧远程桥接器也不自动认识这两个新方法，请在板上使用本包CLI。

## 4. 源码修改清单

| 文件 | 修改 |
|---|---|
| `duck-control/src/lib.rs` | 导出新增构型模块 |
| `duck-control/src/morphology.rs`，新增 | 配置解析/校验、真实ID和逻辑槽映射、固定观测/命令/目标、写入打包、温度掩码、状态报告；13项Rust单元测试 |
| `duck-control/src/bus.rs` | 18项精确替换：初始化、ping、寄存器检查、替换电机、启动位置读取、快慢读取、目标/扭矩/增益写入、重启都使用真实成员 |
| `robotd/src/control.rs` | 11项精确替换：固定观测和有效命令，保持14维原始历史，过滤后固定虚拟目标，屏蔽整机技能 |
| `robotd/src/headless.rs`，新增 | 控制权限检查、故障锁止/手动复位、本地诊断方法；2项Rust单元测试 |
| `robotd/src/main.rs` | 17项精确替换：启动配置、服务/通知门禁、循环安全处理、最终目标投影、物理温度统计、重启全部电机、能力报告 |
| `robotd/tests/headless_profile.rs`，新增 | 3项进程隔离集成测试，覆盖两种构型、Full15兼容及错误配置启动拒绝 |

未修改：`model.rs`、`obs.rs`、`policy.rs`、共享协议、共享参数类型、`Cargo.toml`、`Cargo.lock`、任何ONNX。

全部47项替换的**修改前/修改后代码及逐项原因**在 `CHANGE_REVIEW.zh-CN.md`。应用器会核对4个原始文件的Git blob SHA和每个锚点出现次数；没有匹配则停止，不猜测新版本位置。

### 对原始行为的显式改变

缺失头颈模式屏蔽头部/注视点控制、身体姿态模式、翻滚/拾取/踢球/坐起等整机技能和行走/轮滑切换。保留真实嘴部时，嘴部命令仍可用。速度保持原控制方式，但增加可配置的试验速度上限，默认 `[0.10,0.04,0.40]`，单位为m/s、m/s、rad/s；这不是已验证速度性能。

原有动作缩放、腿部低通和增益公式不改。5V稳压电源场景需额外关闭原2S电池电压补偿和低电量自动关机假设，见配置章节。

在缺失头颈模式下，启动寄存器检查将 `shutdown` 的输入电压故障位保留为1（原52变53）。这是明确的保护性改动，不是策略参数。它**不代表电机可以使用超额定电压**，也不保证在6V立即保护；本包没有把默认电压阈值重设为6V。完整15舵机模式保持原代码行为。

## 5. 故障和关机行为

原版跌倒标志本身不禁止继续推理；包含恢复能力的原策略可能尝试起身。本包在头颈缺失模式下：

- 已收敛IMU下出现原有跌倒判定，或总线连续失败达到原 `max_consecutive_errors`，会锁止运动。
- 由拥有总线的控制线程尝试关闭所有真实舵机扭矩；失败会重试，不伪报已关闭。
- 锁止时丢弃待处理init，拒绝重新enable，不自动坐下/起身。它是跌倒后的停止机制，不是经验证的提前防摔系统；断扭矩本身会令无支撑机器人倒下。
- 默认保留原IMU重复块**诊断**，不把“数值重复”自动等同于“采样停止”。`stop_on_identical_imu_blocks=null`。只有实测确认不会被静止量化误触发后，才能显式设置至少25的重复块锁止门槛。
- 复位要求新鲜且直立、静止的样本、无总线失败、IMU就绪、策略关闭，并在已有锁止时确认扭矩关闭成功。复位只清锁，不打开扭矩、不启动策略。

锁止是进程内状态，不跨进程重启保存。配置变更和重启必须在有机械支撑、动力关闭的条件下操作，不能将重启当作故障复位捷径。

关机不再调用未经适配的坐下网络，而沿用原程序的“没有坐下策略”分支关闭扭矩后关机。关机前必须托住机器。

## 6. 在开发机应用源码包

需要Python 3.11+、Git、Rust 1.89+。下文的 `KIT` 为本包解压位置；`REPO` 必须是官方Rust控制仓库，不是训练仓库或教程仓库。

### 新建独立工作目录

```bash
export KIT="$HOME/Downloads/microduck-headless-v015-r1"
export REPO="$HOME/microduck-headless-v015"

git clone https://github.com/pollen-robotics/microduck.git "$REPO"
git -C "$REPO" switch -c headless-v015 \
  a9ec4b2079ef8ee7904014089c885bb07d57d63c
```

已有官方clone时，可改用独立worktree，不动原main：

```bash
cd /你现有的官方microduck目录
git fetch origin tag daemon-v0.15.0
git worktree add -b headless-v015 "$REPO" \
  a9ec4b2079ef8ee7904014089c885bb07d57d63c
```

以上两种任选其一，不要连续运行建立同名分支。

### 检查、应用、审阅

```bash
python3 "$KIT/apply.py" --repo "$REPO" --check
python3 "$KIT/apply.py" --repo "$REPO"
git -C "$REPO" diff --stat
git -C "$REPO" diff --check
```

注意：新文件属于未追踪文件，普通 `git diff` 不会列出；先直接阅读payload和代码评审文档。确认后再暂存这7个源码文件：

```bash
cd "$REPO"
git add duck-control/src/lib.rs duck-control/src/bus.rs \
  duck-control/src/morphology.rs robotd/src/main.rs \
  robotd/src/control.rs robotd/src/headless.rs \
  robotd/tests/headless_profile.rs
git diff --cached --stat
git diff --cached
```

应用器在本包目录生成完整 `generated.patch`，其中包含新文件。它只在读取到你的完整、正确基线后生成；交付包中的 `edits.json` 是精确修改配方，不是假冒已验证的统一diff。

### 原生测试与构建

```bash
# macOS/Linux：在本机运行Rust测试和本机robotd构建，不接真实电机
bash "$KIT/tools/verify.sh" "$REPO"

# Linux开发机额外运行PTY模拟串口，仍不接真实电机
bash "$KIT/tools/verify.sh" "$REPO" --pty
```

脚本内部使用原生：

```bash
cargo test --locked -p duck-control -p robotd
cargo build --locked -p robotd --bin robotd
```

Linux依赖请按固定版本仓库 `CONTRIBUTING.md` 安装。不要为了通过构建随意更新Cargo.lock或升级全部依赖。

PTY测试会实际启动编译后的robotd，检查10/11设备列表、原15槽scatter、输出gather、没有TorqueEnable=1、缺失ID拒绝，以及真实腿部ID13掉线后出现总线故障。它只实现普通Sync Read，测试TOML关闭Fast Sync Read；**不验证0x8A，也不是IMU固件或物理仿真**。

格式化建议在首次应用成功后单独执行 `cargo fmt --all` 并审阅变化。格式化会改变精确后像，因此之后再次运行本包应用/反向应用可能被保护性拒绝；格式化后的撤销请使用专用Git分支的提交回退，而不是强制跳过校验。

### 构建给Zero 3W使用的程序

macOS产物不能直接复制到Linux板上，即使都是ARM64。使用仓库原生交叉编译别名（需要已安装cargo-zigbuild与Zig）：

```bash
cd "$REPO"
rustup target add aarch64-unknown-linux-gnu
cargo board --locked -p robotd --bin robotd
file target/aarch64-unknown-linux-gnu/release/robotd
```

该别名在固定源码 `.cargo/config.toml` 中是 `zigbuild --release --target aarch64-unknown-linux-gnu.2.31`。构建结果必须是AArch64 Linux ELF。

也可在已有完整源码和Rust工具链的Zero 3W上原生构建：

```bash
cargo build --release --locked -p robotd --bin robotd
file target/release/robotd
```

只有通过测试后才构建/安装候选版本。交付包没有ARM预编译二进制。

## 7. 配置的完整含义

本包 `profiles/headless-mouth.json`：

```json
{
  "schema_version": 1,
  "profile": "headless",
  "mouth_present": true,
  "locked_head_rad": [0.3491, 0.3491, 0.0, 0.0],
  "locked_mouth_rad": 0.0,
  "allow_motion": false,
  "max_twist": [0.10, 0.04, 0.40],
  "stop_on_identical_imu_blocks": null
}
```

`mouth_present=false` 就是双腿10舵机。`allow_motion=false` 拒绝init/enable与硬件扭矩开启，但**不是纯只读模式**：原启动流程仍会检查/修正寄存器，循环仍可能写保持目标。初次测试务必先断动力、支撑并检查舵机状态。

`robotd.toml` 仍采用原0.15.0格式。生成一个保留其他设置的候选副本：

```bash
python3 "$KIT/tools/prepare_config.py" \
  --input /etc/robot/robotd.toml \
  --output "$HOME/robotd.headless.candidate.toml" \
  --regulated-5v

diff -u /etc/robot/robotd.toml "$HOME/robotd.headless.candidate.toml" || true
```

**`--regulated-5v` 仅用于舵机由5V稳压器供电的机器**：关闭 `policy.voltage_adapt` 和 `safety.battery_empty_shutdown`。稳压后的舵机电压不等于电池电量；原电量显示仍可能显示0%，本补丁没有发明新的电池测量。必须另行保留电池BMS/欠压保护及供电测量。不要为迎合原策略电压模型而给XL330过压供电。

自制IMU固件或舵机不支持Fast Sync Read时，加：

```text
--plain-sync-read
```

不加时保留原配置。标准读取和快速读取必须分别在实际固件上验证。

生成器会固定50Hz、walk模式，关闭内置坐起/拾取/踢球/翻滚槽，关闭原自动limp→起身流程，**保留既有walk、stand路径，以及动作缩放、低通和增益**。没有显式设置的旧参数仍由同一0.15.0默认值解析。已有自定义技能表不删除，但本补丁不会执行或将其列为可用技能。

需要指定你原来已在使用的某个ONNX，可加 `--walk /板上的实际绝对路径/original.onnx`。生成器不会下载、修改或替换权重。不要盲填不存在的 `alpha_walking.onnx` 文件名。

## 8. 安装到已经正常部署0.15.0的板子

**以下是首次安装本实验补丁的流程**，不运行provision、不重装系统、不重置其他守护进程、不覆盖官方release目录。包和已构建AArch64二进制先传到板上，例如：

```bash
# 在开发机执行，替换地址
scp "$REPO/target/aarch64-unknown-linux-gnu/release/robotd" 用户@板子地址:~/robotd-headless
scp -r "$KIT" 用户@板子地址:~/
```

在板上设置路径，先确认动力关闭且机械支撑完成：

```bash
export KIT="$HOME/microduck-headless-v015-r1"
file "$HOME/robotd-headless"
python3 --version

# 不覆盖你之前已有的同名实验配置
for p in /etc/systemd/system/robotd.service.d/90-headless.conf \
 /etc/systemd/system/updaterd.service.d/90-headless-lab.conf; do
  test ! -e "$p" || { echo "已有配置，请先审阅: $p"; exit 1; }
done
test ! -e /etc/robot/HEADLESS_LAB_ALLOW_OFFICIAL_UPDATES || exit 1

export BACKUP="/var/backups/microduck-headless/$(date +%Y%m%d-%H%M%S)"
sudo mkdir -p "$BACKUP"
sudo cp -a /etc/robot/robotd.toml "$BACKUP/robotd.toml"
if [ -e /etc/robot/morphology.json ]; then
  sudo cp -a /etc/robot/morphology.json "$BACKUP/morphology.json"
fi
systemctl is-active updaterd.service | sudo tee "$BACKUP/updaterd-active.txt" || true
systemctl is-enabled updaterd.service | sudo tee "$BACKUP/updaterd-enabled.txt" || true
systemctl cat robotd.service | sudo tee "$BACKUP/robotd-unit.txt" >/dev/null
printf '%s\n' "$BACKUP" > "$HOME/headless-backup-path.txt"
```

记录当前权重校验值，下面以标准目录为例；存在自定义路径的也要记录：

```bash
find -L /opt/robot/policies/current -type f -name '*.onnx' \
  -exec sha256sum '{}' \; > "$HOME/headless-weights.before.sha256"
```

现在停止服务并安装候选程序。暂停官方updater是为了避免“官方release更新了，但systemd仍跑自定义二进制”的版本错配。会同时暂停该守护进程提供的其他相关功能，测试期间不使用更新/模型安装命令。

```bash
sudo systemctl stop robotd.service updaterd.service
sudo install -d /opt/robot/local/headless-v015-r1 \
  /etc/systemd/system/robotd.service.d \
  /etc/systemd/system/updaterd.service.d
sudo install -m 0755 "$HOME/robotd-headless" /opt/robot/local/headless-v015-r1/robotd

# 仅缺4个且嘴存在：
sudo install -m 0644 "$KIT/profiles/headless-mouth.json" /etc/robot/morphology.json
# 只有双腿10个时，上面一行改为：
# sudo install -m 0644 "$KIT/profiles/legs10.json" /etc/robot/morphology.json

sudo install -m 0644 "$HOME/robotd.headless.candidate.toml" /etc/robot/robotd.toml
sudo install -m 0644 "$KIT/deploy/robotd-90-headless-bench.conf" \
  /etc/systemd/system/robotd.service.d/90-headless.conf
sudo install -m 0644 "$KIT/deploy/updaterd-90-headless-lab.conf" \
  /etc/systemd/system/updaterd.service.d/90-headless-lab.conf
sudo systemctl daemon-reload
sudo systemctl start robotd.service
```

updater暂停使用一个不存在路径的systemd启动条件，不依赖`mask`覆盖已位于`/etc/systemd/system`的本地unit。不要创建条件文件。回滚时只移除本包的drop-in，不删除原unit。

### 第一阶段：bench，不加载策略、不使能扭矩

候选unit含 `--no-policy`，JSON含 `allow_motion=false`。主控服务启动后，电机动力仍须按照有支撑的台架流程恢复，检查TorqueEnable及寄存器状态，而不是立即让机器人落地。

```bash
sudo python3 "$KIT/tools/morphology_ctl.py" status
robotctl health
sudo journalctl -u robotd.service -n 100 --no-pager
```

必须看到 `patch=headless-v015-r1`，正确的10/11个 `active_motor_ids`，4/5个 `virtual_motor_ids`，61和14的策略宽度，以及明确的虚拟状态说明。没有这个patch标识就不要继续：只看到版本0.15.0不能证明已运行修改版。

`robotctl health` 只是运行与总线健康，不是平衡证明；跌倒锁止状态另看 `robot.morphology`。

### 第二阶段：检查原ONNX

在已安装numpy和onnxruntime的Python环境中，例如你的 `microduck_rl` 环境：

```bash
uv run python "$KIT/tools/check_onnx.py" /实际位置/original.onnx
```

该工具只针对单输入/单输出的前馈61→14策略，执行10次CPU推理，并核对前后文件SHA256相同。多输入的循环策略请沿用原daemon加载器与对应状态测试，不要为了通过此工具修改网络。

它不证明内部归一化语义、物理稳定性或延迟正确。完整硬件闭环验收必须使用原daemon实际ONNX加载路径。

### 第三阶段：在防坠支撑下允许策略

确认真实腿部映射、IMU方向/更新、供电、HOME和限位后，在动力关闭且完全支撑下：

```bash
sudo systemctl stop robotd.service
sudo python3 - <<'PY'
import json, os, tempfile
from pathlib import Path
p=Path('/etc/robot/morphology.json')
d=json.loads(p.read_text()); d['allow_motion']=True
fd,name=tempfile.mkstemp(prefix='.morphology-',dir=p.parent)
with os.fdopen(fd,'w') as f:
    json.dump(d,f,indent=2); f.write('\n'); f.flush(); os.fsync(f.fileno())
os.chmod(name,0o644); os.replace(name,p)
PY
sudo install -m 0644 "$KIT/deploy/robotd-90-headless.conf" \
  /etc/systemd/system/robotd.service.d/90-headless.conf
sudo systemctl daemon-reload
sudo systemctl start robotd.service
```

同时确认原TOML的 `[policy] enabled=true`；这是允许加载策略，不是自动打开扭矩。上面的第二个unit移除了 `--no-policy`。首次由原手柄Start显式启用，先在支撑下检查小幅动作，再进行受保护站立。不要使用无支撑的“直接走几步试试”。

检查原权重未变化：

```bash
sha256sum -c "$HOME/headless-weights.before.sha256"
sudo python3 "$KIT/tools/morphology_ctl.py" status
```

### 故障复位

修好故障并托住机器，确认策略关闭、真实传感器就绪、机体直立静止后：

```bash
sudo python3 "$KIT/tools/morphology_ctl.py" rearm
sudo python3 "$KIT/tools/morphology_ctl.py" status
```

`accepted` 只表示已排队。必须确认 `fault_latched=false`；复位不会上扭矩或启动策略，还需要下一次明确的init/Start。真实舵机未恢复、扭矩关闭没有确认时不会清锁。

## 9. 手柄与旧客户端的兼容限制

原速度命令、停止和手柄连接机制保留。头部/身体姿态命令在robotd入口拒绝，通知也不执行；技能列表和订阅能力不再列出不可用的整机技能。

本包**没有修改padd的本地摇杆模式状态机**。旧padd的Y/B仍可能将摇杆切换到头部/身体模式；此时相关命令被拒绝，不代表腿部映射坏了。需要切回原行走模式再用摇杆移动。旧页面仍可能画出完整15关节模型，虚拟槽不是真实电机测量。

保留的 `robot.model` / 相机ToF正运动学仍基于官方几何。如果你拆掉或移动头部传感器，不可把它当成新相机外参；这不属于本次腿部策略兼容修改。

## 10. 回滚

### 源码回滚

未格式化、没有后续手工改动时：

```bash
python3 "$KIT/apply.py" --repo "$REPO" --reverse --check
python3 "$KIT/apply.py" --repo "$REPO" --reverse
```

若源码已格式化/修改，精确校验会拒绝。用专用Git分支按提交撤销；不要用强制覆盖绕过本包的保护。

### 板端回滚

先机械支撑并关闭电机动力。读取 `~/headless-backup-path.txt`，确认是本次备份：

```bash
BACKUP=$(cat "$HOME/headless-backup-path.txt")
sudo systemctl stop robotd.service
sudo cp -a "$BACKUP/robotd.toml" /etc/robot/robotd.toml
sudo rm /etc/systemd/system/robotd.service.d/90-headless.conf
if [ -e "$BACKUP/morphology.json" ]; then
  sudo cp -a "$BACKUP/morphology.json" /etc/robot/morphology.json
else
  sudo rm /etc/robot/morphology.json
fi
sudo systemctl daemon-reload
```

**不要马上启动原版robotd：四个头颈电机仍然缺失时，原版15舵机程序仍不兼容。** 保持动力关闭、服务停止；只有恢复完整硬件或安装另一个已经验证的兼容版本后再启动。原始release和权重始终未被覆盖。

确认已恢复匹配的完整软件/硬件后，才移除更新暂停并根据备份恢复updater运行状态：

```bash
sudo rm /etc/systemd/system/updaterd.service.d/90-headless-lab.conf
sudo systemctl daemon-reload
# 仅当备份记录它原先为active、且已恢复正常版本关系时：
# sudo systemctl start updaterd.service
```

## 11. 最终验收条件

软件：精确基线应用通过；Rust测试、原生构建、ARM构建通过；10/11构型实际串口读写只有正确成员；右腿回填原逻辑槽10..14；头颈观测固定、原始历史14维不改；真实腿掉线不补零；缺失ID重启拒绝；旧ONNX的SHA256不变。

硬件：供电符合电机规格；头部机械固定或移除且质量变化已评估；IMU是真实且方向正确；先吊架验映射，再保护站立，再低速前后/侧移/转向；测试停车、通信丢失和故障复位。每一步保存日志与结果，失败不靠提高电压、增益或放松故障检测掩盖。

通过这些条件仍只说明已完成你所测工况的验收，不等于所有地面、负载、姿态和速度都安全。
