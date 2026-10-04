# Zero3W 11 舵机候选：阅读入口

本目录是此分支适配机制的唯一文档来源。工作区 `/Users/bou/Workspace/Duckduck/docs/zero3w-11servo` 是带哈希清单的导出副本；请先修改仓库文档，再运行 `tools/zero3w/export_docs.py`，避免两份说明漂移。

基线固定为 `a9ec4b2079ef8ee7904014089c885bb07d57d63c`（v0.15.0）。开发分支 `zero3w/v015-11servo`，审查比较分支 `zero3w/v015-base`。本轮仅交付候选代码、离线验证和迁移准备；不合并、不发布正式版本、不执行带动力测试。

| 要理解的内容 | 文档（机制所有者） |
|---|---|
| 为什么 10 个腿自由度仍使用 15 槽、61/14 ABI；r1 到 r2 的修改理由 | [改造审查](review.zh-CN.md) |
| 每个 ID/逻辑槽/动作索引；真实数据与虚拟数据的边界 | [硬件映射](hardware-map.zh-CN.md) |
| 不训练可复用什么；九工况、额外转换工况和模型哈希 | [模型能力与仿真](model-capabilities.zh-CN.md) |
| 哪些测试通过，哪些受环境影响，哪些还没做 | [验证报告](validation.zh-CN.md) |
| 只读预检、备份、候选安装、fake验收与安全回退 | [迁移与回滚](migration.zh-CN.md) |
| 供电、机械、零位、IMU、质量与分级验收 | [验收检查表](acceptance.zh-CN.md) |
| 如何构建、重跑测试、整理操作审计 | [可复用工作流](workflow.zh-CN.md) |

流程图同时提供 Mermaid 源码和 SVG：[图册](diagrams/README.md)。机器可读结果与候选清单位于 `evidence/`；完整原始输出由本地 `logs/<RUN_ID>` 保留，公开分支只收录不含私有端点或凭证的汇总。

已知限制：新的只读检查已核对板端基线；运动策略目录缺失，原控制循环未启动，供电未验收。实时组件证据和 Runtime 区分见[验证报告](validation.zh-CN.md)。实现完成不等于已能真实行走，请以验证报告和能力矩阵为准。
