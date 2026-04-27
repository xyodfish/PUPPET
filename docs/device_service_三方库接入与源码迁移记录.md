# Device Service 三方库接入与源码迁移记录

## 读者可见变化

- 把 device 侧从“单 demo 可执行程序”收敛成“可插拔、可复用、统一配置”的服务化入口。
- 保留 embosa / zmq 多通信后端并行支持，不移除 embosa 依赖。
- 接入 `scaled_device`，采用**源码迁移 + SDK 直连**方案，不依赖 OmniLink 上层库（如 `ScaledDevice` / `OmnilinkManager`）。

## 总体设计

统一入口二进制：

- `app/cpp/devices/device_service_main.cpp`

可复用核心实现（放在 `include/src`）：

- `include/puppet/device/*`（device service + provider 抽象）
- `include/puppet/device/providers/*`（不同设备 provider）
- `include/puppet/transport/*`（device output channel）
- `src/cpp/puppet/device/*`
- `src/cpp/puppet/device/providers/*`
- `src/cpp/puppet/transport/*`

抽象分层：

- `IDeviceProvider`：设备数据源插件（生成 `PrimitiveFrame`）
- `IDeviceOutputChannel`：通信发布插件（embosa/zmq，放在 `transport` 层）
- `DeviceService`：主循环调度（provider + channel）

数据边界：

- provider 的 `nextFrame(...)` 只填充 `model::PrimitiveFrame`
- protobuf 封装只在 channel `publish(...)` 内处理

## Provider 列表

- `static_file_replay`
  - 对应旧 `static_file_replay_device*` 逻辑
  - 从 human frame json 回放，生成 `PrimitiveFrame.poses`
- `single_chain_ik_sender`
  - 发送逻辑对齐 `embosa_single_chain_ik_sender`
  - 只替换通信出口，轨迹/seed 关节内容保持一致
- `scaled_device`
  - 通过 `GalbotRemoteOperate` SDK 直接收主端数据
  - 迁移核心映射逻辑到 `PrimitiveFrame.joint_states`
  - 不依赖 OmniLink 的 `ScaledDevice/OmnilinkConfig/OmnilinkTypes` 库

## 通信后端

- `embosa`：`transport::EmbosaDeviceOutputChannel`
- `zmq`：`transport::ZmqDeviceOutputChannel`

由配置项 `channel.type` 选择：

- `embosa`
- `zmq`

## 三方库接入

### 1) 保留依赖

- embosa 相关依赖保持不变（未清理）
- zmq 相关依赖保持不变

### 2) scaled_device SDK

在 `src/cpp/CMakeLists.txt` 增加默认 SDK 路径：

- `SCALED_DEVICE_SDK_ROOT`（默认）
  - `${REPO_ROOT}/third_party/galbot_remote_operate`
- 头文件与库缺失时直接 CMake 报错（默认必须有 SDK）
- 不再使用 `PUPPET_WITH_SCALED_DEVICE_SDK` 条件编译宏

并链接 SDK so：

- `libgalbotRemoteOperate.so`
- `libgalbotSystem.so`
- `libgalbotSerialPort.so`
- `libgalbotDataProtocol.so`
- `libgalbotLog.so`

说明：

- 这是**设备 SDK 层依赖**，不是 OmniLink 上层业务库依赖。
- SDK 已迁入仓库 `third_party/galbot_remote_operate`，仅保留 Galbot SDK 相关头文件和 so。
- `yaml-cpp` 未随 SDK 拷贝，继续使用系统/本地 `find_package(yaml-cpp)`。

## galbot_one_golf 关节名对齐

对照 OmniLink 配置：

- `/home/yuxia/Workspace/SingoriX/OmniLink/singorix_omnilink/config/task_config/galbot_one_golf/config_for_galbot_one_golf_real.yaml`

已将 `scaled_device` 相关配置从示例关节名替换为真实命名：

- 左臂：`left_arm_joint1..left_arm_joint7`
- 右臂：`right_arm_joint1..right_arm_joint7`
- 夹爪：`left_gripper_joint1`、`right_gripper_joint1`

对应文件：

- `config/device/device_service_scaled_embosa.yaml`
- `config/device/device_service_scaled_zmq.yaml`

## 配置文件

统一配置样例：

- `config/device/device_service_static_file_embosa.yaml`
- `config/device/device_service_static_file_zmq.yaml`
- `config/device/device_service_single_chain_ik_embosa.yaml`
- `config/device/device_service_single_chain_ik_zmq.yaml`
- `config/device/device_service_scaled_embosa.yaml`
- `config/device/device_service_scaled_zmq.yaml`

统一配置关键字段：

- 顶层：`node_name/topic_name/source_id/frame_id/loop_hz`
- `channel.type`：`embosa|zmq`
- `device.type`：`static_file_replay|single_chain_ik_sender|scaled_device`

## 启动脚本

新增：

- `scripts/start_single_chain_ik_modular_device_service_embosa.sh`
- `scripts/start_single_chain_ik_modular_device_service_zmq.sh`

默认切换到统一服务：

- `scripts/start_retargeting_3nodes.sh`
- `scripts/start_retargeting_3nodes_zmq.sh`

二者默认 `DEVICE_BIN` 均改为：

- `build/app/cpp/devices/device_service`

## 兼容性说明

- 旧可执行文件保留：
  - `static_file_replay_device`
  - `static_file_replay_device_zmq`
- embosa 依赖保留，不影响现有 embosa 通路。
- 新架构通过配置选择 device/channel，减少新增设备时对 app 层的侵入。
