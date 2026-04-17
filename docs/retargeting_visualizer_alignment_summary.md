# Retargeting Visualizer 对齐问题总结

本文档总结了从 `./scripts/start_retargeting_3nodes.sh` 与 `main_retarget_viewer.cpp` 显示不一致开始，到最终完成对齐的关键问题、原因和解决方案。

## 1. 下肢动作异常、整体姿态不对

### 现象
- 三节点脚本能驱动 MuJoCo 机器人运动，但下肢和整体动作与 `main_retarget_viewer.cpp` 明显不一致。

### 原因
- 初始链路中仅传输了标量关节值（hinge/slide），没有传输 free joint（root）7 维 `qpos`（`x y z qw qx qy qz`）。
- `main_retarget_viewer.cpp` 使用的是完整 `qpos`，两者状态不等价。

### 解决
- 在 `GmrRetargetingPlugin` 输出的 `JointCommandIntent` 中追加 root 7 维数据。
- 在 `retargeting_mujoco_visualizer` 中解析 root 7 维并写回 MuJoCo free joint 对应 `qpos`。

---

## 2. MuJoCo 渲染状态更新策略不一致

### 现象
- 即使关节值接近，画面行为仍与 `main_retarget_viewer.cpp` 有差异。

### 原因
- visualizer 每帧状态同步细节与 `main_retarget_viewer.cpp` 不一致。

### 解决
- 对齐为与 `main_retarget_viewer.cpp` 同样流程：
  1. 每帧先 `mju_copy(data->qpos, model->qpos0, model->nq)`；
  2. 再应用目标关节/root `qpos`；
  3. 对 limited hinge/slide 做 `jnt_range` 限幅；
  4. 最后 `mj_forward`。

---

## 3. 缺少 human overlay

### 现象
- 三节点 visualizer 仅显示机器人，不显示人体参考。

### 解决
- 新增 `PrimitiveFrame` 订阅（默认 `puppet_demo/primitive_frame`）。
- 将每个 human pose 渲染为 RGB 三轴箭头 overlay。

---

## 4. human overlay 比例/位置与 `main_retarget_viewer.cpp` 不一致

### 现象
- human 箭头与机器人相对比例和位置不对。

### 原因
- 初版 human overlay 直接画原始 `PrimitiveFrame`，而 `main_retarget_viewer.cpp` 画的是 `prepareHumanFrame(...)` 之后的结果（含 scale/offset/ground 处理）。

### 解决
- 在 visualizer 增加 human overlay 预处理：
  - 使用 `gmr::Retargeter::prepareHumanFrame(...)`；
  - 配置 `ik_config`、`actual_human_height`、`offset_to_ground`；
  - 再绘制处理后的 human frame。

---

## 5. 画面看起来左右镜像（第一帧方向相反）

### 现象
- 你期望“左转打拳”，但脚本显示“右转打拳”。

### 原因
- 相机参数未对齐：visualizer 默认强制了 `camera_azimuth: 180.0`，而 `main_retarget_viewer.cpp` 默认不强制该值。

### 解决
- visualizer 改为“仅在显式配置 `camera_azimuth` 时才覆盖相机方位”。
- 默认配置中移除 `camera_azimuth`，回归与 `main_retarget_viewer.cpp` 一致的默认行为。

---

## 6. `retargeter_mujoco_legacy` 本地 retarget 与 `ControlIntent` 显示不同步

### 现象
- 本进程 retarget 结果与接收的 `ControlIntent` 渲染结果存在相位差（通信延迟）。

### 原因
- 双链路：
  - 链路 A：本地进程内 retarget（即时）；
  - 链路 B：runtime -> 通信 -> visualizer 的 `ControlIntent`（存在延迟）。
- 两条链路天然不同步。

### 解决
- 在 visualizer 增加 `robot_qpos_source`：
  - `control_intent`：沿用旧方案；
  - `local_retarget`：visualizer 内直接对 `PrimitiveFrame` 做 retarget 并渲染（用于消除通信延迟相位差）。
- 默认切到 `local_retarget`，并支持 `mujoco_jacobian_legacy` 配置。

---

## 7. 新增/关键配置项

`config/tools/retargeting_mujoco_visualizer.yaml` 已扩展支持以下配置：

- `robot_qpos_source`: `control_intent | local_retarget`
- `human_frame_topic_name`
- `show_human_overlay`
- `human_overlay_scale`
- `human_overlay_use_prepared_frame`
- `human_overlay_backend`
- `human_overlay_robot_model_path`
- `human_overlay_ik_config_path`
- `human_overlay_actual_human_height`
- `human_overlay_offset_to_ground`
- `local_retarget_backend`
- `local_retarget_robot_model_path`
- `local_retarget_ik_config_path`
- `local_retarget_actual_human_height`
- `local_retarget_damping`
- `local_retarget_max_iterations`
- `local_retarget_use_velocity_limit`
- `local_retarget_integration_timestep`
- `local_retarget_progress_threshold`
- `local_retarget_offset_to_ground`

---

## 8. 结论

本轮改动后，`retargeting_mujoco_visualizer` 已从“仅消费控制意图的简化显示器”升级为可与 `main_retarget_viewer.cpp` 对齐的显示链路，关键在于：

- 对齐状态同步语义（完整 `qpos`、`qpos0` 重建、限幅）；
- 对齐人体可视化语义（`prepareHumanFrame`）；
- 对齐相机默认行为（不强制 azimuth）；
- 支持本地 retarget 模式以规避通信延迟造成的相位不一致。

