# Single-Chain IK + MuJoCo Demo

## 目标
- 输入：`PrimitiveFrame` 中 `right_wrist` 末端位姿
- 运行时：`single_chain_ik`（TRAC-IK）转成 `ControlIntent` 关节目标
- 可视化：MuJoCo 订阅 `control_intent` 显示机器人动作，并叠加输入人端位姿

## 依赖
- 已能构建并运行：
  - `teleop_runtime_embosa_main`
  - `retargeting_mujoco_visualizer`
  - `test_embosa_single_chain_ik_sender`

## 配置文件
- Runtime（单文件）: `config/runtime/demo_single_chain_ik_runtime.yaml`
- Runtime（模块化）: `config/runtime/demo_single_chain_ik_runtime_modular.yaml`
  - 其中 `module_configs` 已包含 `gmr_plugin` 配置文件，默认 `enabled: false`，可直接切换到 true 使用。
- Visualizer: `config/tools/demo_single_chain_ik_visualizer.yaml`

## 运行步骤（三终端）
1. 终端 A：启动 runtime
```bash
./build/app/cpp/runtime/teleop_runtime_embosa_main config/runtime/demo_single_chain_ik_runtime.yaml
# 或模块化配置
# ./build/app/cpp/runtime/teleop_runtime_embosa_main config/runtime/demo_single_chain_ik_runtime_modular.yaml
```

2. 终端 B：启动 MuJoCo 可视化
```bash
./build/app/cpp/tools/retargeting_mujoco_visualizer config/tools/demo_single_chain_ik_visualizer.yaml
```

3. 终端 C：启动 PrimitiveFrame sender
```bash
./build/test/demos/cpp/test_embosa_single_chain_ik_sender
```

## 观测点
- 机器人右臂会在可达空间内做小幅圆轨迹。
- 终端 A 若出现 IK 失败日志，优先检查：
  - `ee_entity` 是否对齐：`right_wrist`
  - `base_link/tip_link` 是否匹配 URDF
  - 输入位姿是否超出可达范围（可先减小轨迹半径）
