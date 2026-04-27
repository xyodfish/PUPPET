# 🎬 Retargeting 3-Nodes Demo

这个 demo 用 3 个进程串起一条最小可运行链路：

```text
device_service(static_file_replay)
  -> (PrimitiveFrame)
teleop_runtime_embosa_main
  -> (ControlIntent)
retargeting_mujoco_visualizer
```

用于快速验证：

- `PrimitiveFrame -> GMR Retargeting -> ControlIntent` 是否正常
- MuJoCo 机器人显示是否与 `main_retarget_viewer.cpp` 对齐
- human overlay 与 robot motion 是否一致

---

## 1) 编译

在仓库根目录执行：

```bash
cmake -S . -B build
cmake --build build -j"$(nproc)"
```

或使用：

```bash
./build.sh
```

---

## 2) 运行

默认运行（使用仓库内默认配置）：

```bash
./scripts/start_retargeting_3nodes.sh
```

带自定义配置运行：

```bash
./scripts/start_retargeting_3nodes.sh \
  /abs/path/runtime.yaml \
  /abs/path/device.yaml \
  /abs/path/viewer.yaml
```

---

## 3) 这个脚本做了什么

脚本路径：`scripts/start_retargeting_3nodes.sh`

按顺序启动 3 个节点：

1. `teleop_runtime_embosa_main`  
   读取 runtime 配置，消费 `PrimitiveFrame`，执行 retargeting/pipeline，发布 `ControlIntent`。

2. `static_file_replay_device`  
   从 JSON 回放 human frame，发布 `PrimitiveFrame`。  
   当前脚本默认使用统一入口 `device_service` + `device_service_static_file_embosa.yaml`。

3. `retargeting_mujoco_visualizer`  
   订阅并渲染机器人动作（可选本地 retarget 模式），同时绘制 human overlay。

脚本会把日志写到：

- `bin/log/teleop_runtime_embosa_main.log`
- `bin/log/device_service_static_file_embosa.log`
- `bin/log/retargeting_mujoco_visualizer.log`

按 `Ctrl+C` 可一次性停止全部节点。

---

## 4) 关键配置（viewer）

文件：`config/tools/retargeting_mujoco_visualizer.yaml`

常用项：

- `robot_qpos_source`: `control_intent | local_retarget`
- `topic_name`: ControlIntent 订阅 topic
- `human_frame_topic_name`: PrimitiveFrame 订阅 topic
- `show_human_overlay`: 是否绘制 human overlay
- `human_overlay_use_prepared_frame`: 是否先走 `prepareHumanFrame`
- `camera_body_name` / `camera_distance` / `camera_azimuth`（可选）

说明：

- 推荐 `robot_qpos_source: local_retarget`，可减少通信延时导致的相位差。
- 不显式配置 `camera_azimuth` 时，使用与 `main_retarget_viewer.cpp` 一致的默认相机行为。

---

## 5) 常见问题

### 机器人左右看起来像镜像

- 先检查是否配置了 `camera_azimuth`。
- 若要与 `main_retarget_viewer.cpp` 默认一致，移除该项或保持未设置。

### human overlay 比例不对

- 开启 `human_overlay_use_prepared_frame: true`。
- 对齐 `human_overlay_ik_config_path` 与 `human_overlay_actual_human_height`。
- `human_overlay_scale` 只影响箭头长度，不影响人体点位比例。

### 运行时动作与本地 retarget 不同步

- 使用 `robot_qpos_source: local_retarget`，避免 `ControlIntent` 通信链路延迟导致的相位差。
