# Runtime Channel ZMQ 接入说明

> 适用日期：2026-04-24  
> 目标：在保留 embosa 通信链路的前提下，新增 zmq 通信链路，支持按配置切换。

## 1. 三方库依赖

- 依赖根路径：`/opt/robot/devel/x86_64-Linux-GNU-9.4.0`
- 头文件：`include/zmq.h`
- 动态库：`lib/libzmq.so`

工程 CMake 已接入：
- `src/cpp/CMakeLists.txt` 增加 `ZMQ_INCLUDE_DIR`、`ZMQ_LIBRARY_DIR`、`libzmq.so` 链接。
- `app/cpp/devices/CMakeLists.txt`、`app/cpp/tools/CMakeLists.txt` 增加 ZMQ 目标的 include/link/rpath。
- `test/demos/cpp/CMakeLists.txt` 增加 `test_zmq_single_chain_ik_sender`。

## 2. Runtime 层改动

新增配置：
- `include/puppet/transport/zmq_runtime_config.hpp`
- `src/cpp/puppet/transport/zmq_runtime_config.cpp`

新增通信实现：
- `include/puppet/transport/zmq_runtime_channel.hpp`
- `src/cpp/puppet/transport/zmq_runtime_channel.cpp`

配置与实例化接线：
- `include/puppet/config/puppet_config.hpp`
- `src/cpp/puppet/config/puppet_config.cpp`
- `src/cpp/puppet/core/puppet_manager.cpp`

说明：
- `runtime_channel.type=embosa`：走原 embosa 实现。
- `runtime_channel.type=zmq`：走 `ZmqRuntimeChannel`。
- embosa 代码路径未删除，可并行维护。

## 3. 配置文件

新增 runtime 模块配置：
- `config/runtime/modules/demo_single_chain_ik/zmq_runtime.yaml`

新增 modular runtime 配置：
- `config/runtime/demo_single_chain_ik_runtime_modular_zmq.yaml`

新增工具/发送端配置：
- `config/tools/retargeting_mujoco_visualizer_zmq.yaml`
- `config/tools/demo_single_chain_ik_visualizer_zmq.yaml`
- `config/device/static_file_replay_device_zmq.yaml`

## 4. 可执行与脚本

### 4.1 新增 ZMQ 可执行

- `build/app/cpp/devices/static_file_replay_device_zmq`
- `build/app/cpp/tools/retargeting_mujoco_visualizer_zmq`
- `build/test/demos/cpp/test_zmq_single_chain_ik_sender`

### 4.2 新增脚本

- `scripts/start_retargeting_3nodes_zmq.sh`  
  用于 runtime + zmq sender + zmq viewer 三节点启动。

- `scripts/start_single_chain_ik_modular_embosa.sh`  
  用于 `demo_single_chain_ik_runtime_modular.yaml`，sender 使用 `test_embosa_single_chain_ik_sender`。

- `scripts/start_single_chain_ik_modular_zmq.sh`  
  用于 `demo_single_chain_ik_runtime_modular_zmq.yaml`，sender 使用 `test_zmq_single_chain_ik_sender`。

## 5. 单链 IK sender 对齐说明

`test_zmq_single_chain_ik_sender` 与 `test_embosa_single_chain_ik_sender` 对齐策略：

- PrimitiveFrame 字段构造逻辑一致：
  - `source_id`: `demo_sender_single_chain_ik`
  - `pipeline_id`: `single_chain_ik_pipeline`
  - 左右臂末端轨迹、seed joint state、tag 填充一致
- 发布频率一致：20ms（约 50Hz）
- 差异仅在传输层：
  - embosa 版：embosa writer
  - zmq 版：ZMQ PUB（topic + proto payload）

## 6. 使用建议

### embosa modular

```bash
./scripts/start_single_chain_ik_modular_embosa.sh
```

### zmq modular

```bash
./scripts/start_single_chain_ik_modular_zmq.sh
```

`start_single_chain_ik_modular_zmq.sh` 参数：
1. runtime 配置路径（默认 `demo_single_chain_ik_runtime_modular_zmq.yaml`）
2. sender endpoint（默认 `tcp://127.0.0.1:16661`）
3. viewer 配置路径（默认 `demo_single_chain_ik_visualizer_zmq.yaml`）

## 7. 已知边界

- embosa sender 与 `runtime_channel.type=zmq` 不互通。
- zmq sender 与 `runtime_channel.type=embosa` 不互通。
- 若遇到 runtime 启动报 output endpoint 异常，优先检查端口占用（例如 `16663` 被其他进程 bind）。
