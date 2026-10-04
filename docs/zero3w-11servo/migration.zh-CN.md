# 分步迁移与回滚：本轮硬件服务保持停止

当前状态：目标SSH在认证前关闭，以下板端步骤尚未执行成功。脚本语法/输入/审计测试通过不等于已在目标systemd验收。只有自审、人的review及板端只读身份核对通过后安装候选；本轮不能开启硬件服务。

## 统一审计调用

所有 SSH/SCP 都必须经 `tools/zero3w/deploy.py`，内部使用 `audit_command.py`。每条远端命令启动前保存 argv/UTC/意义，执行后保存完整输出、退出码、输出哈希；失败、断线也保留。发送脚本及配置复制工具按内容哈希归档在 `logs/<RUN_ID>/scripts/<SHA>/`；二进制上传记录本地路径、来源提交、SHA256及目的路径。

不输出密码、token、全环境或账号文件。backup配置及unit原文只保存在板端0700独立目录，公开输出为状态、路径和哈希。不要用裸ssh/scp补做未审计操作；如需诊断，先写精确脚本并通过同一审计器归档执行，写明意义及后续注意事项。

在独立仓库根目录运行（目标与RUN_ID需按本地操作记录填写，真实端点不放公开仓库）：

```bash
export TARGET='root@zero3w'
export RUN_ID='zero3w-11servo-20261004T014630+0800'
export AUDIT_ROOT="/Users/bou/Workspace/Duckduck/logs/$RUN_ID"
python3 tools/zero3w/deploy.py preflight --target "$TARGET" --run-id "$RUN_ID" --audit-root "$AUDIT_ROOT"
```

### 1. 只读预检

**意义**：核对root/AArch64、系统服务运行/使能状态、主进程实际exe及哈希、有效配置、drop-in路径、daemon manifest身份、全部ONNX哈希、Runtime库哈希及其 C API实际版本。Python>=3.11、systemd与SSH host-key校验为前提；不自动安装前提工具，不降低host-key检查。

**预期**：退出0且得到JSON快照。核对基线a9ec4b2...的可验证manifest/build provenance，若仅有版本0.15.0仍不足以证明具体提交。核对robotd真实参数路径是否为默认 `/etc/robot/robotd.toml`、服务文件与drop-in覆盖是否改变实际启动路径/配置/库搜索路径。预检不打印ExecStart/environment，必要时以追加审计的白名单字段查询补证，不能盲用默认路径。

**停止条件**：SSH错误、身份/架构错误、有效配置来源不确定、实际部署提交无法核实、策略路径/Runtime有差异、官方自动更新或启动覆盖未识别。当前正停在SSH失败。**恢复**：由现场/网络管理员恢复该已确认端点的SSH，保留失败记录；重新只读预检，不换目标猜测。

### 2. 准备独立目录与备份

先确认review通过及电机供电已由人在场物理隔离。`--power-isolated` 不是软件探测，不能由时间或fake结果代替。先用 `--plan-only` 查看计划；其不执行SSH，只归档精确脚本。

```bash
python3 tools/zero3w/deploy.py prepare --target "$TARGET" --run-id "$RUN_ID" --audit-root "$AUDIT_ROOT"
python3 tools/zero3w/deploy.py backup --target "$TARGET" --run-id "$RUN_ID" --audit-root "$AUDIT_ROOT" --preflight-reviewed --power-isolated
```

**意义/预期**：独占新建 `/opt/robot/local/zero3w-11servo/$RUN_ID`，0700权限；保存ownership、before.json、原配置/官方robotd副本及本地unit/drop-in副本，ONNX逐文件哈希在快照内。模型不复制或改写。原服务状态原样记录，不能在回退时默认恢复active。

**停止条件**：本次目录已存在、备份失败、供电未隔离、预检未review。**恢复**：读取本次ownership与日志判断完成到哪一步；不覆盖旧目录，用新RUN_ID继续经过审查的重试。私有备份需另做受控离机备份，不能把含配置原文的整个目录上传公共仓库。

### 3. 安装候选（不启动）

```bash
export SOURCE_COMMIT='<候选清单中40位源码提交>'
python3 tools/zero3w/deploy.py install --target "$TARGET" --run-id "$RUN_ID" --audit-root "$AUDIT_ROOT" \
  --binary target/aarch64-unknown-linux-gnu/release/robotd --source-commit "$SOURCE_COMMIT" \
  --preflight-reviewed --power-isolated
```

**意义/预期**：先审计SCP，上传独立staged文件；远端逐字节校验二进制及配置复制工具哈希，确认官方文件未自备份后漂移。先检查没有预存在的审批marker/owned drop-in/unit；停止原robotd/updaterd，添加本次专属ConditionPathExists防自动启动/更新。原二进制、原配置、原模型路径保持；候选生成独立robotd.toml，walk/stand、HOME、滤波、增益/缩放保留，整机技能配置关闭。候选 morphology为11真实设备且allow_motion=false。

安装 `zero3w-11servo-candidate.service`，其ConditionPathExists指向不存在的硬件审批marker，Restart=no，无开机enable。**候选服务停止**，不创建任何marker，不调用init/enable；最终退出0、installed.json记录所有本次owned systemd路径及SHA。默认使用原总线协议配置，不能把PTY通过当实际Fast固件通过。

**停止条件**：备份不完整、哈希错误、文件漂移、owned路径冲突、服务停止/写入失败。**恢复**：电机供电保持隔离，按operation与远端实际文件/哈希核对部分完成状态。失败不重跑覆盖；如未生成installed.json，不自动rollback，先对本次专属路径审查后通过审计脚本清理。不得删除不属于本次的drop-in或重启原15舵机服务。

### 4. 隔离fake验收

```bash
python3 tools/zero3w/deploy.py fake --target "$TARGET" --run-id "$RUN_ID" --audit-root "$AUDIT_ROOT"
```

**意义/预期**：候选只以 `--fake --no-policy` 启动；临时socket、runtime、参数均独立，不占原/run/robotd.sock、不打开总线、不读取原audio/policy。验证61/14、11ID、头颈/技能/enable/init拒绝，退出后fake进程终止，原硬件服务持续停止。板端保留fake-runtime.log和fake-result JSON，stdout由本地审计保存。

**停止条件**：候选ELF不能加载、隔离路径冲突、fake断言失败/超时。**恢复**：fake进程由finally结束；保持硬件服务停止，查询必要身份/动态库信息也必须审计。此步不推断ARM真实ONNX推理时延；实际策略文件取得后另跑隔离模型契约/有限推理，不能把 --no-policy 说作模型已验证。

### 5. 回退与恢复边界

```bash
python3 tools/zero3w/deploy.py rollback --target "$TARGET" --run-id "$RUN_ID" --audit-root "$AUDIT_ROOT" --preflight-reviewed --power-isolated
```

仅核对并移除本次候选unit，保留本次目录的日志/备份；原文件从未覆盖，因此不以旧副本盲覆盖后来的其他修改。owned systemd文件必须与installed.json哈希一致，漂移即停止审查。硬件缺失头颈时保留本次原robotd/updaterd防启动hold，返回JSON明确这是**保留安全门禁的部分回退**，不会宣称已经完全恢复原运行状态。

若已由人在场机械恢复全部15关节并核验，单次回退可追加 `--full15-restored` 移除本次两个hold。依旧不启动任何服务；原active状态仅用于后续人工审查决定，不能自动重启。不要在11舵机硬件上创建 ZERO3W_FULL15_RESTORED；不要为继续测试创建候选审批marker。本轮没有实机启动步骤。

**停止条件**：没有installed.json、owned文件哈希不匹配、供电仍接通、无法确认本次目录。**恢复**：保留证据、支撑并切断动力；人工审查明确本次改动列表后，仅通过审计执行对应恢复，不运行官方update.rollback来绕过构型约束。

后续带动力验收必须满足 [检查表](acceptance.zh-CN.md) 和独立review，单独审查审批marker、allow_motion、稳压后的电压适应/电池阈值、服务恢复及策略来源。这些步骤本轮未执行。
