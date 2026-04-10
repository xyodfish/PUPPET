# PUPPET Proto Overview And Semantics

## 1. 总体目标

这套 proto 用于把遥操作链路拆成稳定分层：

```text
Source Input
  -> PrimitiveFrame
  -> Orchestrator / Retargeting
  -> ControlIntent
  -> Robot Backend
```

若输入不是单帧流，而是轨迹，则走：

```text
PrimitiveFrameTrajectory
  -> Sampler(t)
  -> PrimitiveFrame
```

## 2. 文件职责

## 2.1 [common.proto](/home/yuxia/Workspace/PUPPET/proto/proto/puppet/common.proto)

基础类型，不携带业务语义：

- 时间：`Timestamp`, `Duration`
- 通用头：`Header`
- 几何：`Point`, `Vector3`, `Quaternion`, `Pose`, `Twist`

## 2.2 [uniform.proto](/home/yuxia/Workspace/PUPPET/proto/proto/puppet/uniform.proto)

跨层统一元信息：

- 控制分组：`BodyGroup`
- 来源分类：`SourceType`
- 帧上下文：`FrameContext`
- primitive 元数据：`PrimitiveMeta`

## 2.3 [primitive_frame.proto](/home/yuxia/Workspace/PUPPET/proto/proto/puppet/primitive_frame.proto)

输入语义层（设备语义层）：

- 顶层：`PrimitiveFrame`
- typed repeated primitives：
  - `PosePrimitive`
  - `TwistPrimitive`
  - `JointStatePrimitive`
  - `JointCommandPrimitive`
  - `ScalarPrimitive`
  - `BooleanPrimitive`
  - `ModePrimitive`
  - `PlanarMotionPrimitive`
  - `SkeletonPrimitive`
  - `LandmarkSetPrimitive`

## 2.4 [control_intent.proto](/home/yuxia/Workspace/PUPPET/proto/proto/puppet/control_intent.proto)

机器人控制语义层：

- 顶层：`ControlIntent`
- 分组聚合：`GroupControlIntent`
- 子意图：
  - `CartPoseIntent`
  - `JointCommandIntent`
  - `PostureIntent`
  - `BaseMotionIntent`
  - `ConstraintRequest`

## 2.5 [primitive_trajectory.proto](/home/yuxia/Workspace/PUPPET/proto/proto/puppet/primitive_trajectory.proto)

非单帧输入支持：

- 时间域：`TimeDomain`
- 插值策略：`InterpolationMode`
- 轨迹头：`TrajectoryHeader`
- 采样点：`TimedPrimitiveFrame`
- 轨迹：`PrimitiveFrameTrajectory`

## 3. FrameContext 语义（重点）

`FrameContext` 的目标是告诉运行时：这帧“来自谁、在什么场景、用哪种模式处理”。

### 3.1 `semantic_context` 是什么

`semantic_context` 表示“语义场景/任务域”，决定系统在宏观上处于哪类工作流。

建议值与典型场景：

1. `teleop`
- 场景：实时人控。
- 例子：VR 控制器持续输出手腕 pose，驱动机械臂实时跟随。

2. `manual`
- 场景：人工直接接管，尽量少自动化。
- 例子：手柄直接控制底盘速度，不启用复杂辅助约束。

3. `assist`
- 场景：人机协同，算法补完。
- 例子：操作者给粗略末端意图，IK/WBC 处理可达性和约束。

4. `replay`
- 场景：回放历史记录或离线轨迹。
- 例子：从 `PrimitiveFrameTrajectory` 采样重放，用于回归测试。

5. `handover`
- 场景：控制权切换过程。
- 例子：从 `vr_operator_1` 切到 `master_arm_node`，过渡期平滑衔接。

6. `calibration`
- 场景：标定流程。
- 例子：采集固定姿态，估计设备到机器人坐标变换。

### 3.2 `mode` 是什么

`mode` 表示“当前场景下具体采用的控制方式”，比 `semantic_context` 更细粒度。

常用示例：

1. `direct_pose`
- 行为：`PosePrimitive` 近似直接映射到末端目标。
- 特点：低延迟，策略简单，对输入质量敏感。

2. `assist_ik`
- 行为：操作者给意图，IK backend 求解实际关节目标。
- 特点：可达性更好，更容易挂约束（关节限位/碰撞等）。

3. `clutch`
- 行为：离合/解耦。常见为“按住才更新”或“按住时冻结机器人”。
- 特点：便于操作者重新摆姿，避免非期望动作。

4. `trajectory_playback`
- 行为：输入来自轨迹采样器而非实时设备。
- 特点：便于复现、验证、回归测试。

### 3.3 如何组合

- `semantic_context` 决定“在做哪类事情”。
- `mode` 决定“这类事情当前怎么做”。

示例：

- `assist + assist_ik`：协同遥操作 + IK 约束求解。
- `replay + trajectory_playback`：轨迹回放链路。
- `teleop + clutch`：实时遥操作中进入离合态。

## 4. 坐标与时间字段边界（易混）

## 4.1 时间字段

1. `Timestamp`
- 绝对时刻。
- 约定：`nanosec` 在当前秒内。

2. `Duration`
- 时间长度。
- 本项目中既保留 `sec`（便于计算）也保留 `nanosec`（高精度表达）。
- 工程建议：消费侧固定一个主字段（推荐 `sec`）并在接口文档中统一。

3. `header.timestamp` vs `meta.timestamp`
- `header.timestamp`：消息级时间（整帧/整意图）。
- `meta.timestamp`：单 primitive 时间（可精细到不同采样源）。

## 4.2 坐标系字段

1. `Header.frame_id`
- 该消息默认坐标系。

2. `PrimitiveMeta.frame_id`
- 某个 primitive 当前值所在坐标系。

3. `PrimitiveMeta.reference_frame_id`
- 该 primitive 的参考坐标系（常用于相对表达）。

4. `PosePrimitive.target_frame_id`
- 当 `is_relative=true` 时，pose 相对哪个目标坐标系。

5. `TwistPrimitive.body_frame_id` 与 `reference_frame_id`
- `body_frame_id`：速度作用对象。
- `reference_frame_id`：速度表达参考系。

## 5. PrimitiveFrame 使用约定

1. 单帧边界
- 一帧建议只包含“一个 source + 一个时间片 + 一个语义上下文”。

2. `sequence_id`
- 建议在同一 `source_id` 内单调递增，用于去重和乱序恢复。

3. `confidence` 与 `valid`
- `confidence`：质量分数（建议归一化到 `[0, 1]`）。
- `valid=false`：语义存在但数据无效（可用于降级/占位）。

4. repeated 对齐
- 有 `name`/`joint_names` 时，消费侧应按名称对齐，不要仅按索引。

## 6. ControlIntent 使用约定

## 6.1 GroupControlIntent

1. `owner_source_id`
- 当前分组控制权归属来源。

2. `priority`
- 同组多意图冲突时的仲裁优先级（数值越大优先级越高）。

3. `backend_hint`
- 给 backend 选择器的建议，不是强绑定。
- 示例：`DirectMappingBackend`、`IkBackend`、`TsidBackend`。

4. `enabled`
- 分组开关。`false` 时建议下游忽略该分组意图。

## 6.2 ConstraintRequest

1. `hard=true`
- 必须满足；若求解器不可行应触发降级或保护停机策略。

2. `hard=false`
- 软约束，参与代价函数，受 `weight` 影响。

3. `scalar_params` / `string_params`
- 用于扩展参数，避免频繁改 proto。
- 建议做键名约定，例如：
  - `margin`
  - `safe_distance`
  - `max_velocity`

## 7. PrimitiveFrameTrajectory 使用约定

1. `samples` 必须按 `t_from_start` 严格升序。
2. `looping=true` 时建议明确 `total_duration`。
3. `default_interp` 是默认策略，未来可扩展为“按 primitive 类型覆盖”。
4. 若轨迹用于在线控制，推荐 `TIME_DOMAIN_MONOTONIC`。

采样细则见：

- [sampleing_guideline.md](/home/yuxia/Workspace/PUPPET/docs/sampleing_guideline.md)

## 8. tags 的使用边界

`tags` 适合承载：

- 调试标记
- 实验开关
- 统计标签

`tags` 不应承载：

- 关键控制语义
- 必需解析字段

关键语义应优先进入显式 proto 字段，避免隐式约定。

## 9. 推荐开发顺序

1. 打通 `PrimitiveFrame -> ControlIntent` 主链路。
2. 接入轨迹采样 `PrimitiveFrameTrajectory -> Sampler`。
3. 增强后端约束能力（IK/TSID/Optimization）。

## 10. 多数据源仲裁建议

当多个 source 同时给同一身体分组输入时，建议做“按 `BodyGroup` 独立仲裁”。

### 10.1 仲裁流程

1. 预过滤（硬门槛）
- 仅保留 `valid=true` 且未超时的输入。
- `semantic_context` 与当前运行态不匹配时直接丢弃。
- `mode` 不兼容当前 pipeline/back-end 时降权或丢弃。
- `deadman/clutch` 不满足条件时禁止该 source 出控制。

2. 构建分组候选集
- 按 `BodyGroup` 建立候选列表。
- 例如 `LEFT_ARM`、`BASE`、`WHOLE_BODY` 分开处理，不做全局一刀切。

3. 计算分数并选主控
- 为每个候选 source 计算总分：

```text
score = wp*priority + wf*freshness + wc*confidence + wh*health + wm*mode_match
```

- `priority`：配置优先级（静态）。
- `freshness`：时效分（动态，越新越高）。
- `confidence`：输入质量（动态）。
- `health`：source 健康状态（动态）。
- `mode_match`：与当前系统 mode 的匹配程度（动态）。

4. 抖动抑制
- 引入滞回阈值：新 source 分数要高出当前 owner 一个 margin 才允许切换。
- 引入最小持有时间：owner 至少持有 `hold_ms`（如 200~500ms）。

5. 切换平滑（handover）
- 发生 owner 变化时，不建议瞬时切换。
- 对 pose/joint/base 指令做短时 blend（如 100~300ms）。

6. 输出控制意图
- 在 `GroupControlIntent` 中明确填写：
  - `owner_source_id`
  - `priority`
  - `mode`
  - `backend_hint`
  - `enabled`

### 10.2 融合策略建议

1. 可融合项
- 底盘速度（`BaseMotionIntent`）可加权融合。
- 软约束（`ConstraintRequest.hard=false`）可叠加。

2. 不建议直接融合项
- 同一关节位置命令通常不建议直接平均。
- 同一末端位姿若来自不同 source，优先“单 owner + 平滑切换”。

3. 回退链
- owner 失效时按 fallback 顺序接管：
  - `Primary -> Secondary -> SafeHold`
- `SafeHold` 可输出零速度或保持上次稳定目标。

## 11. Orchestrator 伪代码

下面伪代码对应“多 source 输入 -> 分组仲裁 -> 输出 `ControlIntent`”主流程。

```cpp
ControlIntent Orchestrator::Tick(double now_sec) {
  // 1) 收集并更新 source cache
  auto incoming_frames = source_manager_.Collect(now_sec);
  cache_.Update(incoming_frames, now_sec);

  // 2) 解析系统运行态
  RuntimeState rt = runtime_state_.Get();  // current semantic_context/mode/enable flags

  // 3) 按 BodyGroup 做候选筛选与仲裁
  std::unordered_map<BodyGroup, OwnerDecision> owners;
  for (BodyGroup g : configured_groups_) {
    auto candidates = cache_.GetCandidatesForGroup(g);

    // 3.1 硬门槛过滤：valid/fresh/context/mode/deadman
    candidates = FilterCandidates(candidates, rt, now_sec);

    // 3.2 打分
    for (auto& c : candidates) {
      c.score = weights_.wp * c.priority +
                weights_.wf * FreshnessScore(c, now_sec) +
                weights_.wc * ConfidenceScore(c) +
                weights_.wh * HealthScore(c.source_id) +
                weights_.wm * ModeMatchScore(c, rt);
    }

    // 3.3 选择 owner（带滞回和最小持有时间）
    owners[g] = arbitration_.SelectOwner(g, candidates, now_sec);
  }

  // 4) 生成每个 group 的输入并跑 retargeting
  std::vector<GroupControlIntent> group_intents;
  for (BodyGroup g : configured_groups_) {
    auto owner = owners[g];
    auto group_input = BuildGroupInput(g, owner, cache_, rt);
    auto group_intent = pipeline_manager_.Run(g, group_input, rt.mode);

    // 4.1 handover 平滑
    if (arbitration_.OwnerChanged(g)) {
      group_intent = BlendWithPrevious(g, group_intent, blend_window_ms_);
    }

    // 4.2 回填仲裁信息
    group_intent.set_body_group(g);
    group_intent.set_owner_source_id(owner.source_id);
    group_intent.set_priority(owner.priority);
    group_intent.set_mode(rt.mode);
    group_intent.set_enabled(owner.enabled);
    group_intent.set_backend_hint(SelectBackendHint(g, rt.mode));

    group_intents.push_back(std::move(group_intent));
  }

  // 5) 组装顶层 ControlIntent
  ControlIntent out;
  out.mutable_header()->set_frame_id("world");
  FillTimestamp(out.mutable_header()->mutable_timestamp(), now_sec);
  FillFrameContext(out.mutable_context(), rt);
  out.set_sequence_id(seq_++);
  for (auto& gi : group_intents) {
    *out.add_group_intents() = std::move(gi);
  }
  (*out.mutable_tags())["orchestrator"] = "main";

  // 6) 发布/记录/监控
  publisher_.Publish(out);
  recorder_.Write(out);
  monitor_.Update(out, owners, now_sec);

  return out;
}
```

### 11.1 参数建议（首版）

1. `hold_ms`: 300
2. `switch_margin`: 0.1 ~ 0.2（归一化分数域）
3. `blend_window_ms`: 150 ~ 250
4. `source_timeout_ms`: 100 ~ 300（按设备频率调整）
