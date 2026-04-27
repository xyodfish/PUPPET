# Runtime Channel 多传输接入说明（Embosa / ZMQ / TCP / UDP）

> 适用日期：2026-04-27  
> 目标：在保留 embosa 链路的前提下，支持 `zmq/tcp/udp` 运行时通道，并统一三节点 demo 启动方式。

## 1. 三方库与系统依赖

- 依赖根路径：`/opt/robot/devel/x86_64-Linux-GNU-9.4.0`
- 头文件：`include/zmq.h`
- 动态库：`lib/libzmq.so`

工程 CMake 已接入：
- `src/cpp/CMakeLists.txt` 增加 `ZMQ_INCLUDE_DIR`、`ZMQ_LIBRARY_DIR`、`libzmq.so` 链接。
- `app/cpp/devices/CMakeLists.txt`、`app/cpp/tools/CMakeLists.txt` 增加 ZMQ 目标的 include/link/rpath。
- `test/demos/cpp/CMakeLists.txt` 增加 `test_zmq_single_chain_ik_sender`。

## 2. Runtime 层改动

新增配置（ZMQ/TCP/UDP）：
- `include/puppet/transport/zmq_runtime_config.hpp`
- `src/cpp/puppet/transport/zmq_runtime_config.cpp`
- `include/puppet/transport/tcp_runtime_config.hpp`
- `src/cpp/puppet/transport/tcp_runtime_config.cpp`
- `include/puppet/transport/udp_runtime_config.hpp`
- `src/cpp/puppet/transport/udp_runtime_config.cpp`

新增通信实现（`IRuntimeChannel` 派生）：
- `include/puppet/transport/zmq_runtime_channel.hpp`
- `src/cpp/puppet/transport/zmq_runtime_channel.cpp`
- `include/puppet/transport/tcp_runtime_channel.hpp`
- `src/cpp/puppet/transport/tcp_runtime_channel.cpp`
- `include/puppet/transport/udp_runtime_channel.hpp`
- `src/cpp/puppet/transport/udp_runtime_channel.cpp`

配置与实例化接线：
- `include/puppet/config/puppet_config.hpp`
- `src/cpp/puppet/config/puppet_config.cpp`
- `src/cpp/puppet/core/puppet_manager.cpp`

说明：
- `runtime_channel.type=embosa`：走原 embosa 实现。
- `runtime_channel.type=zmq`：走 `ZmqRuntimeChannel`。
- `runtime_channel.type=tcp`：走 `TcpRuntimeChannel`。
- `runtime_channel.type=udp`：走 `UdpRuntimeChannel`。
- embosa 代码路径未删除，可并行维护。

## 3. 配置文件

新增 runtime 模块配置：
- `config/runtime/modules/demo_single_chain_ik/zmq_runtime.yaml`
- `config/runtime/modules/demo_single_chain_ik/tcp_runtime.yaml`
- `config/runtime/modules/demo_single_chain_ik/udp_runtime.yaml`

新增 modular runtime 配置：
- `config/runtime/demo_single_chain_ik_runtime_modular_zmq.yaml`
- `config/runtime/demo_single_chain_ik_runtime_modular_tcp.yaml`
- `config/runtime/demo_single_chain_ik_runtime_modular_udp.yaml`

新增工具/发送端配置：
- `config/tools/retargeting_mujoco_visualizer_zmq.yaml`
- `config/tools/demo_single_chain_ik_visualizer_zmq.yaml`
- `config/tools/demo_single_chain_ik_visualizer_tcp.yaml`
- `config/tools/demo_single_chain_ik_visualizer_udp.yaml`
- `config/device/static_file_replay_device_zmq.yaml`

## 4. 可执行与脚本

### 4.1 新增可执行

- `build/app/cpp/devices/static_file_replay_device_zmq`
- `build/app/cpp/tools/retargeting_mujoco_visualizer_zmq`
- `build/app/cpp/tools/retargeting_mujoco_visualizer_tcp`
- `build/app/cpp/tools/retargeting_mujoco_visualizer_udp`
- `build/test/demos/cpp/test_zmq_single_chain_ik_sender`
- `build/test/demos/cpp/test_tcp_single_chain_ik_sender`
- `build/test/demos/cpp/test_udp_single_chain_ik_sender`

### 4.2 新增脚本

- `scripts/start_retargeting_3nodes_zmq.sh`  
  用于 runtime + zmq sender + zmq viewer 三节点启动。

- `scripts/start_single_chain_ik_modular_embosa.sh`  
  用于 `demo_single_chain_ik_runtime_modular.yaml`，sender 使用 `test_embosa_single_chain_ik_sender`。

- `scripts/start_single_chain_ik_modular_zmq.sh`  
  用于 `demo_single_chain_ik_runtime_modular_zmq.yaml`，sender 使用 `test_zmq_single_chain_ik_sender`。

- `scripts/start_single_chain_ik_modular_3nodes.sh`  
  统一三节点脚本，支持 `embosa/zmq/tcp/udp`：
  - `embosa`: runtime + embosa sender + embosa viewer
  - `zmq`: runtime + zmq sender + zmq viewer
  - `tcp`: runtime + tcp sender + tcp viewer
  - `udp`: runtime + udp sender + udp viewer

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

### 统一三节点启动（推荐）

```bash
./scripts/start_single_chain_ik_modular_3nodes.sh embosa
./scripts/start_single_chain_ik_modular_3nodes.sh zmq
./scripts/start_single_chain_ik_modular_3nodes.sh tcp
./scripts/start_single_chain_ik_modular_3nodes.sh udp
```

> 说明：脚本已加入启动健康检查，若 runtime/viewer/sender 启动失败会立即退出并打印对应日志。

## 7. 单链 IK 稳定性更新

为降低多解分支切换带来的关节跳变，`single_chain_ik` 插件更新如下：

- seed 策略收敛为：`lastSolution -> defaultPosture`（暂不从遥操 input 推断机器人实时状态）。
- 成功求解后使用与上一帧最接近的等价角（`±2pi`）进行连续性选择。
- 连续求解失败达到阈值后，清空 `lastSolution` 标志，避免长期卡在坏 seed。

## 8. 已知边界

- embosa sender 与 `runtime_channel.type=zmq` 不互通。
- zmq sender 与 `runtime_channel.type=embosa` 不互通。
- 若遇到 runtime 启动报 output endpoint 异常，优先检查端口占用（例如 `16663` 被其他进程 bind）。
- `tcp` 链路为点对点连接，启动顺序需要先起 listener（sender/viewer）再起 runtime。
