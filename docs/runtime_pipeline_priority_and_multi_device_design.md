# Runtime Pipeline 优先级与多设备处理设计（讨论稿）

## 1. 背景与目标
- 现状：`group_routing` 基本是 `group -> pipeline` 一对一静态绑定。
- 新需求：
1. 同一个 `group` 可以注册多个 pipeline（如 `gmr` + `single_chain_ik`）。
2. 每个 pipeline 有 `priority`，同组冲突时取高优先级。
3. `PrimitiveFrame` 若是 `whole_body` 语义，强制高于单组优先级。
4. 需要定义“不同 device 来源数据”的统一处理策略。

目标是：在不破坏现有模块边界（source/runtime/retargeting）的前提下，实现可扩展的编排决策层。

## 2. 核心决策模型

### 2.1 统一术语
- `CandidatePlan`：某一帧内，对某个 `body_group` 的候选控制方案。
- `Score`：候选方案综合评分。
- `WinnerPlan`：每个 `body_group` 最终选中的唯一方案。

### 2.2 候选来源
- 对每个 `body_group`，可从多个 pipeline 生成候选：
  - `single_chain_ik_pipeline`
  - `gmr_pipeline`
  - `direct_pass_pipeline`
  - 后续新增 pipeline

### 2.3 冲突裁决（同 group 多 pipeline）
- 基础规则：同组只允许一个 Winner。
- 主规则：按 `priority` 高低选 Winner。
- 并列规则：若优先级相同，按稳定 tie-break：
1. 数据新鲜度高（`age_ms` 更小）优先。
2. pipeline 健康度高（近期成功率）优先。
3. 若仍相同，按固定字典序（确保结果可复现）。

## 3. whole_body 强制优先规则

### 3.1 规则定义
- 当候选方案语义为 `whole_body`（来源可为：
  - `PrimitiveFrame.context/tags` 明示 whole_body
  - `GroupRouting.body_group == whole_body`）
- 则该候选被打上 `whole_body_boost`，其优先级强制高于单组候选。

### 3.2 建议实现
- 计算评分时使用：
  - `effective_priority = base_priority + whole_body_bonus`
- 其中 `whole_body_bonus` 是固定大值（例如 `+10000`），保证覆盖常规优先级范围。

### 3.3 边界约束
- whole_body 方案胜出后，单组方案不再写入同周期输出（避免双写冲突）。
- 对“安全关键组”（如 base/head）可保留白名单覆盖策略（后续可选）。

## 4. 多设备（multi-device）数据处理策略

## 4.1 问题拆解
- 不同 device（VR、mocap、master arm、vision）存在：
  - 更新频率不同
  - 延迟抖动不同
  - 覆盖 group 不同
  - 可信度不同

### 4.2 建议分层
- `SourceManager` 负责“按 source 存储与健康度”。
- `Orchestrator` 负责“按 group 选择数据拥有者（owner）与候选计划”。
- `RetargetingPipeline` 只做算法，不做跨设备策略。

### 4.3 每个 group 的 device 选择（owner 选择）
- 建议在配置中支持：
  - `owner_candidates`
  - `owner_policy`（`fixed` / `priority` / `freshness` / `hybrid`）
- `hybrid` 推荐公式：
  - `owner_score = source_priority + freshness_score + confidence_score - jitter_penalty`

### 4.4 device 与 pipeline 的关系
- 一种 device 可驱动多个 pipeline（比如 VR 可进 IK 也可 direct）。
- 一个 pipeline 可消费多个 device（前提是输入语义兼容）。
- 最终仍落在“同 group 单 Winner”。

### 4.5 时间与同步建议
- 引入 `frame_time_tolerance_ms`（按 group 配置），超窗数据降权或丢弃。
- 保留 `last_good_plan`（短时间 hold），避免瞬时掉帧造成抖动。

## 5. 配置草案（建议）

```yaml
group_routing:
  - body_group: right_arm
    owner_candidates: [vr_main, mocap_backup]
    owner_policy: hybrid
    pipelines:
      - pipeline_id: single_chain_ik_pipeline
        enabled: true
        priority: 200
      - pipeline_id: gmr_pipeline
        enabled: true
        priority: 120

  - body_group: whole_body
    owner_candidates: [mocap_fullbody]
    owner_policy: fixed
    pipelines:
      - pipeline_id: gmr_pipeline
        enabled: true
        priority: 100
        whole_body_override: true
```

说明：
- `priority` 是 pipeline 在 group 维度下的优先级（建议放在 routing 层，而不是全局 pipeline 定义）。
- `whole_body_override: true` 会触发 `whole_body_bonus`。

## 6. 运行时行为（单周期）
1. 对每个 group 选出 owner source（按 owner_policy）。
2. 基于 owner 数据，生成多个 `CandidatePlan`（遍历 enabled pipelines）。
3. 计算 `effective_priority`（含 whole_body bonus）。
4. 每组选 Winner，写入 `ControlIntent`。
5. 记录统计：各 pipeline 命中率、回退次数、丢帧次数。

## 7. 兼容与迁移计划
1. 第一阶段（兼容）：
  - 保留旧 `group_routing.pipeline` 写法。
  - 新写法存在时优先使用新写法。
2. 第二阶段：
  - 输出告警提示旧写法将废弃。
3. 第三阶段：
  - 全部迁移到 `pipelines[] + priority` 结构。

## 8. 建议先落地的最小版本（MVP）
1. 同组多 pipeline + `priority` 选高。
2. whole_body bonus 覆盖单组。
3. owner 先保留 `fixed`，多 device 的 `hybrid` 打分后续再加。

这样能最快验证控制链路，复杂融合策略可增量引入。

