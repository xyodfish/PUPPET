# Dual-Source Split Demo

## 目标
- `sourceA` 固定负责 `right_arm`
- `sourceB` 固定负责 `left_arm`
- 两个 sender 都会随机切换 `direct_pass` 和 `single_chain_ik`
- runtime 根据每帧的 `context.plugin_id` 从 `active_plugins` 中选择当前策略

## 依赖
- `teleop_runtime_embosa_main`
- `retargeting_mujoco_visualizer`
- `test_embosa_direct_pass_sender`
- `test_embosa_left_arm_single_chain_ik_sender`

## 配置文件
- Runtime: `config/runtime/demo_dual_source_mixed_runtime.yaml`
- Visualizer: `config/tools/demo_single_chain_ik_visualizer.yaml`

## 一键启动
```bash
./scripts/start_dual_source_split_demo.sh
```

## 手动运行步骤（四终端）
1. 启动 runtime
```bash
./build/app/cpp/runtime/teleop_runtime_embosa_main config/runtime/demo_dual_source_mixed_runtime.yaml
```

2. 启动可视化
```bash
./build/app/cpp/tools/retargeting_mujoco_visualizer config/tools/demo_single_chain_ik_visualizer.yaml
```

3. 启动右臂 sender
```bash
./build/test/demos/cpp/test_embosa_direct_pass_sender
```

4. 启动左臂 sender
```bash
./build/test/demos/cpp/test_embosa_left_arm_single_chain_ik_sender
```

## 观测点
- 右臂和左臂都会间歇性切换策略
- sender 日志里的 `strategy=direct_pass_plugin` 表示当前发关节命令
- sender 日志里的 `strategy=single_chain_ik_plugin` 表示当前发末端位姿目标
