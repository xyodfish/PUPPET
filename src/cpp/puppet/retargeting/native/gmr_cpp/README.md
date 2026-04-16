# GMR C++ Retargeting（实验版）

这个目录是 GMR retargeting 的 C++ baseline 实现。

## 已实现内容
- 与 Backend 解耦的 `Retargeter` 基类，输出目标关节坐标（`qpos` stream）。
- 四个具体 retarget Backend：
  - `PinocchioRetargetBackend`
  - `PinocchioLegacyRetargetBackend`
  - `MujocoRetargetBackend`
  - `MujocoLegacyRetargetBackend`
- Backend 选择与渲染目标解耦（MuJoCo / ROS / 其他 GUI）。
- 复用 `whole_body_control` 中优化风格的 QP solver 结构：
  - `qp_solver` / `hqp_solver` / `qp_data`
- 复用现有 `general_motion_retargeting/ik_configs/*.json` 的 IK config。
- 单帧 retarget CLI：`gmr_retarget_cli`。
- 带 YAML 运行配置的 MuJoCo viewer：`gmr_retarget_viewer`（仅渲染）。

## 依赖
默认 prefix：
- `/opt/robot/devel/x86_64-Linux-GNU-9.4.0`

必需 package：
- `Eigen3`
- `qpOASES`
- `pinocchio`
- `mujoco`
- `nlohmann_json` header（`nlohmann/json.hpp`）
- `yaml-cpp`（`yaml-cpp/yaml.h`）

## 编译
```bash
cd /data/open_src_code/GMR_custom
cmake -S cpp -B cpp/build \
  -DGMR_THIRDPARTY_PREFIX=/opt/robot/devel/x86_64-Linux-GNU-9.4.0 \
  -DGMR_MUJOCO_PREFIX=/opt/robot/devel_control/x86_64-Linux-GNU-9.4.0
cmake --build cpp/build -j
```

## 运行 retarget，并打印/保存 qpos
```bash
/data/open_src_code/GMR_custom/cpp/build/gmr_retarget_cli \
  --backend pin_ik \
  --gmr_root /data/open_src_code/GMR_custom \
  --robot unitree_g1 \
  --human_frame_json /data/open_src_code/GMR_custom/cpp/examples/human_frame_smplx_g1_example.json \
  --actual_human_height 1.7 \
  --damping 0.5 \
  --max_iter 10 \
  --use_velocity_limit \
  --out_json /data/open_src_code/GMR_custom/tmp/gmr_cpp_qpos.json
```

## 用 YAML config 运行 viewer（默认 realtime）
```bash
/data/open_src_code/GMR_custom/cpp/build/gmr_retarget_viewer \
  --backend pin_ik \
  --config /data/open_src_code/GMR_custom/cpp/examples/retarget_viewer_config.yaml
```

Backend 名称：
- `pin_ik`（aliases: `pinocchio`, `pinocchio_ik`）
- `pin_ik_jacobian_legacy`（aliases: `pinocchio_legacy`, `pin_legacy`）
- `mujoco_se3`（aliases: `mujoco`, `se3`）
- `mujoco_jacobian_legacy`（aliases: `mujoco_legacy`, `legacy`）

命令行参数会覆盖 YAML 配置，例如：
```bash
/data/open_src_code/GMR_custom/cpp/build/gmr_retarget_viewer \
  --config /data/open_src_code/GMR_custom/cpp/examples/retarget_viewer_config.yaml \
  --precompute
```

## 当前范围
- CLI 在多帧 JSON 输入时只使用第一帧。
- 当前目标域是 SMPL-X 风格的人体 body name（与现有 IK json 对齐）。
- 当前版本是第一版落地，后续可继续迭代 batch mode / pybind / parity tests。
