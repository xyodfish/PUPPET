# PUPPET 架构与算法缺口分析

本文记录当前仓库在架构层和算法层还需要补齐的关键能力。目标不是提出一套大重构方案，而是给后续 PR 排优先级，避免在 demo 能跑之后继续累积隐性复杂度。

## 当前判断

PUPPET 当前已经具备一条可运行主链路：

```text
Device / Source
  -> PrimitiveFrame
  -> RuntimeChannel
  -> SourceManager
  -> GroupRoutingResolver
  -> RetargetingPipeline
  -> ControlIntent
  -> Robot Backend
```

这条链路能支撑单链 IK、direct pass、GMR、mixed-arm split、dual-source split 等 demo。当前更大的风险不在“能不能跑”，而在以下几点：

- 多 source、多 body group、多 plugin 组合之后，runtime 决策规则还不够显式。
- 输入 primitive 的语义约束主要靠配置和插件内部约定，缺少统一校验。
- 算法层还偏 demo 化，实机安全、稳定性和失败恢复策略不足。
- 测试更多依赖脚本和日志，缺少可重复的小单元回归。

## 架构层缺口

### 1. Runtime 主循环职责还需要继续收敛

`TeleopRuntime::runOnce()` 已经开始按阶段拆成 `collectPlanInputs`、`executePlans`、`finalizeControlIntent`。这个方向是对的，但后续如果加入 source arbitration、frame fusion、控制权切换、fallback 等逻辑，主循环仍然容易重新变重。

建议：

- 保持主循环只描述阶段。
- 具体策略优先放在阶段内部的小函数，而不是继续堆在 `runOnce()`。
- 不急着新增 manager / policy 类，除非多个调用点真正共享同一套策略。

### 2. Group routing 缺少完整仲裁模型

当前路由基本是：

```text
body_group -> owner_source -> plugin -> backend
```

这能支撑单 source 或清晰拆分 source 的 demo，但真实遥操作会出现多个 source 同时想控制同一个 body group，例如：

- VR 和主从臂同时控制右臂。
- 自动规划想接管某个 group。
- 保护控制或急停逻辑需要覆盖普通输入。
- 某个 source 超时后需要 fallback 到另一个 source。

建议优先明确这些规则：

- `priority` 如何影响同 group 多 source 选择。
- freshness timeout 后是否 hold last、drop group，还是 fallback。
- clutch / takeover / emergency stop 的优先级。
- 多 source 同时有效时，是互斥选择、融合，还是按 primitive 类型拆分。

### 3. PrimitiveFrame 语义约束还不够强

当前 `PrimitiveFrame` 通过以下字段组合表达输入语义：

- `PrimitiveMeta.body_group`
- `FrameContext.plugin_id`
- `FrameContext.group_plugin_ids`
- `GroupRoutingConfig.control_semantics`
- primitive 类型，例如 pose / twist / joint command

这套表达已经能覆盖 mixed-arm 场景，但缺少统一校验。例如：

- `direct_pass` 应只消费 joint command 类输入。
- `single_chain_ik` 应主要消费 pose / twist 类输入。
- 同一 group 内出现不兼容 primitive 时，应该明确报错、忽略还是降级。

建议：

- 在 runtime 或 plugin dispatch 边界做最小输入 contract 校验。
- 校验错误需要指向 `source_id`、`body_group`、`plugin_id` 和 primitive 类型。
- 避免在每个 plugin 内重复写一套不一致的防御逻辑。

### 4. RetargetingPipeline 命名与当前实现存在语义差

`RetargetingPipeline` 当前更像 plugin dispatcher：根据 `plugin_id` 找到一个 plugin 并执行。它还不是“多个 plugin 递次 process”的 pipeline。

当前处理方式：

- 保留类名 `RetargetingPipeline`，因为它仍是 retargeting 执行入口。
- 输入消息和 YAML 不再使用 `pipeline_id`，统一使用 `plugin_id`。
- 文档中明确 `RetargetingPipeline` 是执行组件名，不是输入消息里的 ID 语义。

后续只有在真的支持多个 plugin 串联时，再考虑把配置层引入 pipeline 概念。不要提前恢复 `pipeline_id`。

### 5. 配置 schema 还没有固化

YAML 当前已经能表达 source、routing、plugins、backend、plugin 参数，但缺少 schema 级校验。

常见风险：

- `group_routing[].plugin` 指向不存在的 plugin。
- `active_plugins[].plugin_id` 和 `plugins[].plugin_id` 不一致。
- provider 发出的 `group_plugin_ids` 不在当前 group 的 `active_plugins` 里。
- `control_semantics` 和 plugin 类型不匹配。

建议：

- 先在 `RuntimeConfigLoader` 或 runtime init 阶段补最小一致性检查。
- 错误信息要定位到 YAML 字段，不要只说 configure failed。
- 短期保留旧 key fallback，但仓库内新配置只使用 `plugin_id/plugins`。

### 6. 测试体系偏弱

当前更可靠的验证方式是构建和跑 demo 脚本。这个能发现集成问题，但不适合防回归。

建议优先补这些小单测：

- `RuntimeConfigLoader`：新旧 key、缺失 plugin、disabled plugin。
- `GroupRoutingResolver`：plans cache、updateGroupRouting 后刷新、priority 排序。
- `proto_copy`：`plugin_id/group_plugin_ids` 双向转换。
- `TeleopRuntime` plugin selection：group override 优先于 frame 默认 plugin。
- group filtering：同一 frame 中 left/right primitive 不互相污染。

## 算法层缺口

### 1. 单链 IK 还偏 demo 化

当前单链 IK 能完成基本 pose 到关节的映射，但实机稳定性还需要补：

- joint limit margin，而不是只依赖硬上下限。
- singularity 附近的阻尼或降权。
- seed 管理策略，例如 last solution、default posture、robot state。
- 失败时 fallback，例如保持上一帧或输出空 intent。
- 速度、加速度、jerk 的平滑和限幅。

### 2. Plugin 输入 contract 需要更明确

这次已经在 runtime 侧做了按 group 过滤，避免 direct pass 把其它 group 的 pose 抄进自己的 group intent。但长期不能只靠“当前 demo 不污染”。

建议：

- 每个 plugin 明确支持的 primitive 类型和 `control_semantics`。
- dispatch 前做一次轻量检查。
- plugin 内只处理自己声明支持的 primitive。

### 3. 多目标融合能力不足

真实控制经常同时包含多个目标：

- wrist pose
- elbow hint
- joint seed
- gaze / torso hint
- collision avoidance
- joint posture regularization

当前更多是 group-wise 单目标 retargeting。后续需要明确目标权重、约束优先级和失败策略。

### 4. 缺少真正 whole-body / QP / TSID 层

当前链路主要是 group-wise retargeting 后拼 `ControlIntent`。这对 demo 友好，但距离全身协调控制还差一层 solver/backend。

需要补齐的问题：

- 多 group intent 如何合成一个一致的全身目标。
- 不同 group 的约束冲突如何处理。
- joint limit、velocity limit、contact、self-collision 如何统一进入 solver。
- backend 是直接映射、IK、QP、TSID 还是 WBC，需要更清晰的职责边界。

### 5. 时序处理不足

输入 frame 高频但可能 jitter、丢帧、延迟或不同 source 不同步。当前主要依赖 freshness timeout，算法层还缺少：

- timestamp-aware interpolation。
- hold-last / drop / fallback 策略。
- deadband 和 rate limit。
- low-pass filter。
- latency compensation。

这些不建议一次性做成大框架，应该先从最容易出问题的 source 和 group 开始局部补。

### 6. 实机安全约束需要系统化

demo 能动不等于能上实机。需要优先明确：

- joint position / velocity / acceleration limit。
- workspace limit。
- self-collision / environment collision。
- emergency stop 在 runtime、backend、device 哪一层生效。
- stale robot state 时是否允许继续执行某些 plugin。

安全约束不应该散在各个 demo sender 里，应该逐步收敛到 runtime/backend/plugin 边界。

## 建议优先级

### P0：先稳住语义和回归

- 补 `RuntimeConfigLoader` schema 校验。
- 补 `proto_copy`、routing、plugin selection 单测。
- 明确 `plugin_id/group_plugin_ids` 是输入消息层唯一 ID 语义。
- 保持 mixed-arm 和 dual-source demo 作为 push 前集成验证。

### P1：补 runtime 决策能力

- 明确同 group 多 source 仲裁规则。
- 补 freshness / fallback / takeover 行为。
- 给 group filtering 和 active plugin selection 增加可测路径。

### P2：补算法稳定性

- 单链 IK 加 joint limit margin、seed 策略和失败 fallback。
- 对 direct pass、single-chain IK 做输入 contract 校验。
- 引入最小平滑和 rate limit。

### P3：再考虑更大 solver/backend 能力

- 多目标融合。
- whole-body QP / TSID / WBC。
- collision constraint。
- 更完整的 robot safety layer。

## Push 前建议验证

涉及 proto、provider、runtime config 或 plugin selection 的改动，至少跑：

```bash
./auto_build.sh --proto-only --install-proto
./build.sh
timeout 12s ./scripts/start_mixed_arm_split_demo.sh
timeout 12s ./scripts/start_dual_source_split_demo.sh
```

如果改过 proto 或 CMake include/link 路径，额外确认运行时加载的是仓库本地 proto 库：

```bash
ldd build/app/cpp/runtime/teleop_runtime_embosa_main | rg "puppet_proto"
```

