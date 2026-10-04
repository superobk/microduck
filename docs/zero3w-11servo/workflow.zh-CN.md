# 可复用构建、验证与审计流程

## 本地准备

独立源码仓库为 `/Users/bou/Workspace/Duckduck/microduck-zero3w-11servo`，原 `/repo/microduck` 与zippack保留。Rust1.89、Python、Zig和缓存均位于工作区 `tools/zero3w-runtime`，没有替换全局Rust或shell配置。Docker Desktop是原安装应用，本轮启动以提供Linux ARM隔离PTY测试；容器不挂载串口，network=none。

```bash
export WORKSPACE='/Users/bou/Workspace/Duckduck'
export CARGO_HOME="$WORKSPACE/tools/zero3w-runtime/cargo"
export RUSTUP_HOME="$WORKSPACE/tools/zero3w-runtime/rustup"
export PATH="$WORKSPACE/tools/zero3w-runtime/venv/bin:$WORKSPACE/tools/zero3w-runtime/venv/lib/python3.12/site-packages/ziglang:$CARGO_HOME/bin:$PATH"
export RUN_ID='zero3w-11servo-20261004T014630+0800'
export AUDIT_ROOT="$WORKSPACE/logs/$RUN_ID"
```

以下命令均包在同一个记录器中：

```bash
python3 tools/zero3w/audit_command.py --root "$AUDIT_ROOT" --meaning '基线和不可变接口核查' --note '有差异即停止审查' -- \
  git diff a9ec4b2079ef8ee7904014089c885bb07d57d63c -- duck-control/src/model.rs duck-control/src/obs.rs duck-control/src/policy.rs duck-ipc-proto robotd-params Cargo.lock
python3 tools/zero3w/audit_command.py --root "$AUDIT_ROOT" --meaning '完整工作区回归' --note '保留环境及断言失败，不静默跳过' -- cargo test --workspace --offline --locked
python3 tools/zero3w/audit_command.py --root "$AUDIT_ROOT" --meaning 'Python工具回归' --note '不连接硬件' -- python3 -m unittest discover -s tools/headless-v015-r1/tests -v
python3 tools/zero3w/audit_command.py --root "$AUDIT_ROOT" --meaning '审计输入保护' --note '不执行远端操作' -- python3 -m unittest discover -s tools/zero3w -v
python3 tools/zero3w/audit_command.py --root "$AUDIT_ROOT" --meaning '日志评估回归' --note '非实机证据' -- python3 -m unittest discover -s tools/zero3w/tests -v
```

本环境唯一DNS/TLS负例说明见验证报告。完整命令失败后，只有为补齐其余结果才允许显式 `--skip turn::tests::a_failure_names_its_cause_and_not_just_itself` 并标记被排除；不能把该运行写成完整通过。真实Runtime测试设置 `ORT_DYLIB_PATH` 后单独运行recurrent_policy的8项 `--ignored` 与控制器反馈重置专用测试，通用工作区不注入Runtime。

## 构建可追踪ARM候选

```bash
export DUCK_REVISION="$(git rev-parse HEAD)"
export DUCK_BUILD_TIME="$(date -u +%Y-%m-%dT%H:%M:%SZ)"
python3 tools/zero3w/audit_command.py --root "$AUDIT_ROOT" --meaning 'AArch64 Linux候选构建' --note '只构建，保持模型不变，不发布' -- \
  cargo board --offline --locked -p robotd --bin robotd
```

产物 `target/aarch64-unknown-linux-gnu/release/robotd`、来源提交、原基线、编译环境和SHA256应纳入候选清单；构建时revision嵌入二进制，后续纯文档提交可能在其后，不能假称二进制包含未构建的代码。保留Cargo.lock，未来重建也锁定依赖，不升级到新主线。

## Linux PTY 与真实推理软件验收

```bash
python3 tools/zero3w/audit_command.py --root "$AUDIT_ROOT" --meaning '普通/Fast协议和10/11/15设备PTY' --note '无实际串口，日志写到本次目录' -- \
  docker run --rm --network none --mount "type=bind,src=$PWD,dst=/src,readonly" \
  --mount "type=bind,src=$AUDIT_ROOT,dst=/results" python:3.12-slim-bookworm \
  python /src/tools/headless-v015-r1/tools/pty_bus_test.py --robotd /src/target/aarch64-unknown-linux-gnu/release/robotd --out /results/linux-pty-new
```

本轮容器digest记在证据清单中。复跑使用新输出名，原日志不得覆盖。`policy_rehearsal.py --help` 提供真实Runtime+fake三阶段参数；只允许--fake隔离，无总线。其专用runtime-nan ONNX是临时测试fixture，保存在logs，不是替换训练模型。

## 复跑参考物理试验

```bash
cargo build --offline --locked -p duck-control --example morphology-golden
python3 tools/zero3w/audit_command.py --root "$AUDIT_ROOT" --meaning '固定头颈参考物理工况' --note '原资源只读，不能代表实际板端模型/身体' -- \
  "$WORKSPACE/tools/zero3w-runtime/venv/bin/python" tools/zero3w/offline_feasibility.py \
  --model "$WORKSPACE/microduck-lab/vendor/microduck-simulator/app/public/robot/mjlab/robot_allcollisions.xml" \
  --meshes "$WORKSPACE/microduck-lab/vendor/microduck-simulator/app/public/robot/mjlab/meshes" \
  --policy-dir "$WORKSPACE/microduck-lab/data/reference" --walk "$WORKSPACE/microduck-lab/data/reference/alpha_walking.onnx" \
  --sitstand "$WORKSPACE/microduck-lab/data/reference/alpha_sitstand.onnx" \
  --golden target/debug/examples/morphology-golden --profile tools/headless-v015-r1/profiles/headless-mouth.json \
  --out "$AUDIT_ROOT/offline-reference-new" --duration 30 --seeds 0 1 2
```

参数、资源哈希、种子、每帧观测/动作/目标、结果与黄金比较分别保存。9基础工况+3转换工况×3种子，另完整构型站立/前进×3对照，共42次。停止/转向判据要求实际测得运动；结论必须分别报告文件接口数量、基础工况仿真数量、实机数量。取得真正板端模型后优先重跑；不引入训练替代当前失败。

## 导出与日志索引

运行 `tools/zero3w/export_docs.py --destination "$WORKSPACE/docs/zero3w-11servo"` 将唯一文档来源复制到用户指定目录并生成导出哈希表；流程图Mermaid/SVG同源由 `render_diagrams.py` 生成。原始日志本地保存，公开版本只放脱敏汇总。每次SSH操作均见迁移指南，不能以只有屏幕输出替代持久审计。
