# Primitive Trajectory Sampling Guideline

## 1. 目标

本文定义 `PrimitiveFrameTrajectory` 在运行时如何被采样成单帧 `PrimitiveFrame`，用于统一接入现有主循环：

```text
Trajectory -> Sampler(t) -> PrimitiveFrame -> Orchestrator -> Retargeting -> ControlIntent
```

## 2. 适用范围

- 适用于 [primitive_trajectory.proto](/home/yuxia/Workspace/PUPPET/proto/proto/puppet/primitive_trajectory.proto)
- 轨迹点为 `TimedPrimitiveFrame`
- 采样输入为时间 `t`（单位秒）

## 3. 前置约束

1. `samples` 必须按 `t_from_start` 严格升序。
2. `samples` 至少包含 1 个点。
3. 相邻点时间差应大于 0。
4. 每个 `PrimitiveFrame` 推荐保持相同 `context.source_id` 与 `context.semantic_context`。

## 4. 时间语义

- `TIME_DOMAIN_MONOTONIC`：推荐用于在线控制，避免墙钟跳变。
- `TIME_DOMAIN_WALL_CLOCK`：用于日志回放或跨进程对时。
- `TIME_DOMAIN_ROBOT_TIME`：用于控制器自身时钟域。

运行时应在 `TrajectoryHeader.time_domain` 下解释 `t_from_start`。

## 5. 采样行为

### 5.1 边界处理

- `t <= t0`：返回第 0 个采样点。
- `t >= tN`：
  - 若 `looping=false`，返回最后一个采样点。
  - 若 `looping=true`，执行 `t = fmod(t, total_duration)` 后再采样。

### 5.2 区间定位

找到满足 `ti <= t < ti+1` 的区间，计算：

```text
alpha = (t - ti) / (ti+1 - ti), alpha ∈ [0, 1]
```

## 6. 插值规则

默认使用 `TrajectoryHeader.default_interp`，也可在未来扩展为“按 Primitive 类型覆盖策略”。

### 6.1 数值类

- `ScalarPrimitive.value`：线性插值
- `PlanarMotionPrimitive(vx, vy, wz)`：线性插值
- `JointStatePrimitive` / `JointCommandPrimitive`：按同名关节逐元素线性插值

### 6.2 位姿类

- `Pose.position`：线性插值
- `Pose.orientation`：`SLERP`（若未实现，先 fallback 到最近邻保持）

### 6.3 速度类

- `Twist.linear` / `Twist.angular`：线性插值

### 6.4 离散类

- `BooleanPrimitive`：保持（`HOLD`）
- `ModePrimitive`：保持（`HOLD`）

### 6.5 结构化集合

- `SkeletonPrimitive`：按 `joint.name` 对齐后插值；缺失关节采用最近邻保持
- `LandmarkSetPrimitive`：按 `landmark.name` 对齐后插值；缺失关键点采用最近邻保持

## 7. 名称对齐与缺失策略

轨迹中涉及 `repeated` 数组时，必须优先做“按名称对齐”，不要只按索引。

推荐策略：

1. 同名项存在于前后两帧：执行插值。
2. 仅存在于前帧或后帧：最近邻保持。
3. 两侧都不存在：跳过。

## 8. 时效性建议

- 采样器输出后建议写入 `PrimitiveFrame.header.timestamp` 为“采样时刻”。
- 若 `t` 与最近轨迹点偏差超过阈值（例如 100ms），建议打告警标签：
  - `frame.tags["sampling_warning"] = "stale_segment"`

## 9. 参考伪代码

```cpp
PrimitiveFrame Sample(const PrimitiveFrameTrajectory& traj, double t_sec) {
  Validate(traj);
  double t = NormalizeTimeDomain(traj.trajectory.time_domain, t_sec);
  t = HandleBoundaryAndLooping(traj, t);
  auto [i, j, alpha] = LocateSegment(traj.samples, t);

  if (traj.trajectory.default_interp == INTERPOLATION_MODE_HOLD) {
    return traj.samples[i].frame;
  }

  PrimitiveFrame out;
  out.header = BuildSampleHeader(t);
  out.context = traj.trajectory.context;
  out.sequence_id = BuildSequenceId();
  InterpolateAllTypedPrimitives(traj.samples[i].frame, traj.samples[j].frame, alpha, &out);
  return out;
}
```

## 10. 实施建议

1. 第一版先支持：`HOLD` + `LINEAR` + Pose 的 `SLERP`。
2. 第二版再加：`Skeleton/Landmark` 名称对齐插值优化。
3. 第三版按业务增加“每类 primitive 独立插值策略”。
