# PUPPET 开发者指南

本文面向需要阅读、扩展或调试 PUPPET 的开发者，重点说明当前仓库真实的模块边界、运行链路、配置方式和常见扩展入口。

## 1. 项目定位

PUPPET 是一套机器人遥操作 runtime 框架。它不把某个输入设备直接绑定到某台机器人，而是先把输入统一成 `PrimitiveFrame`，再通过 runtime 编排、retargeting pipeline 和 robot backend 生成机器人控制语义。

核心目标：

- 统一多种输入源：VR、动捕、主从臂、文件回放、外部消息等。
- 统一设备侧控制语义：用 `PrimitiveFrame` 表达一帧输入。
- 按 body group 和 source ownership 做路由编排。
- 支持可插拔 retargeting：direct pass-through、single chain IK、GMR 等。
- 支持多通信后端：当前主要为 Embosa 和 ZMQ。

## 2. 代码分层

```text
include/puppet/
  common/                 # 时间、几何、统一枚举等基础类型
  primitive/              # PrimitiveFrame / PrimitiveTrajectory C++ model
  control/                # ControlIntent C++ model
  config/                 # 顶层 PuppetConfig
  runtime/                # TeleopRuntime、RobotStateSync、RuntimeConfig
  source/                 # SourceManager
  orchestrator/           # body group routing / plan resolution
  retargeting/            # plugin 接口与具体 retargeting 实现
  robot/backend/          # ControlIntent 到 final target 的映射
  transport/              # Embosa/ZMQ runtime channel 和 device output channel
  device/                 # DeviceService 与 device provider 接口

src/cpp/puppet/
  # 与 include/puppet/ 对应的核心实现

app/cpp/
  runtime/                # runtime 可执行入口
  devices/                # device service / device replay 入口
  tools/                  # visualizer 等工具

config/
  runtime/                # 顶层 runtime 配置和 modular 配置
  device/                 # DeviceService / replay / scaled device 配置
  tools/                  # visualizer 配置
```

分层约束：

- `device` 只负责采集或生成 `PrimitiveFrame`，不要直接耦合 app 或 robot backend。
- `transport` 负责通信和 proto 转换，业务语义应留在 model / runtime 层。
- `runtime` 负责调度和编排，不承载具体算法细节。
- `retargeting` 负责从输入 primitive 转换为 group control intent。
- `app/cpp` 负责拼装配置和启动，不应堆入长期维护的底层逻辑。

## 3. Runtime 数据流

当前主链路由 `PuppetManager` 驱动：

```text
PuppetConfig YAML
  -> PuppetManager::init
  -> create RuntimeChannel (embosa / zmq)
  -> TeleopRuntime::init(RuntimeConfig)
  -> RuntimeChannel::tryPopFrame
  -> SourceManager::captureFrame
  -> Orchestrator::resolvePlans
  -> SourceManager::getLatestFrame(ownerSourceId)
  -> RetargetingPipeline::run
  -> ControlIntent
  -> DirectMappingBackend::buildTarget
  -> RuntimeChannel::publishControlIntent
```

关键类：

- `PuppetManager`：顶层生命周期管理，加载配置，创建通信 channel，注册 robot state handler，驱动主循环。
- `TeleopRuntime`：配置 source、routing、pipeline，并在 `runOnce` 内把最新输入转成 `ControlIntent`。
- `SourceManager`：缓存各 source 的最新 `PrimitiveFrame`。
- `Orchestrator`：根据 `group_routing` 解析每个 body group 的 owner source、pipeline 和 backend。
- `RetargetingPipeline`：按 `pipeline_id` 调用对应 plugin。
- `DirectMappingBackend`：当前 final target 映射入口。

## 4. 配置结构

顶层 runtime 配置通常位于 `config/runtime/*.yaml`，其中 modular 示例会拆分到 `config/runtime/modules/demo_single_chain_ik/`：

- `sources.yaml`：输入源声明，包含 `source_id`、`source_type`、topic、freshness 等。
- `group_routing.yaml`：body group 到 owner source、pipeline、backend 的路由。
- `pipelines.yaml`：pipeline 列表与 plugin 类型。
- `backends.yaml`：backend 列表。
- `single_chain_ik.yaml` / `gmr_plugin.yaml`：具体 retargeting 插件参数。
- `embosa_runtime.yaml` / `zmq_runtime.yaml`：通信后端参数。
- `robot_state.yaml`：robot state freshness 相关参数。

顶层 `PuppetConfig` 中的 `runtime_channel.type` 决定 runtime channel：

```yaml
runtime_channel:
  type: zmq   # embosa | zmq
```

Device Service 配置位于 `config/device/`。关键字段：

- `device.type`：`static_file_replay`、`single_chain_ik_sender`、`scaled_device` 等。
- `channel.type`：`embosa` 或 `zmq`。
- `loop_hz`、`source_id`、`frame_id`、topic/endpoint 等运行参数。

## 5. 构建与依赖

推荐流程：

```bash
./auto_build.sh --proto-only --install-proto
./auto_build.sh --main-only
```

常用覆盖项：

```bash
cmake -S . -B build \
  -DROOT_DEVEL_ARCH_PATH=/opt/robot/devel/x86_64-Linux-GNU-9.4.0 \
  -DSCALED_DEVICE_SDK_ROOT=$PWD/third_party/galbot_remote_operate
cmake --build build -j"$(nproc)"
```

主工程依赖包括 `yaml-cpp`、`Eigen3`、`glog`、`pinocchio`、`qpOASES`、`orocos-kdl`、`trac_ik`、`puppet_proto`、Embosa、ZMQ 和 Galbot Scaled Device SDK。MuJoCo visualizer 还需要 MuJoCo 与 GLFW；缺失时相关工具目标会被跳过或构建失败，具体以 CMake 输出为准。

`puppet_proto` 的 CMake package 默认查找路径与 `auto_build.sh --install-proto` 的默认安装前缀一致：

```text
devel/<system-processor>-Linux-GNU-<compiler-version>
```

## 6. 常见扩展任务

### 6.1 新增输入设备

推荐接入路径：

1. 在 `include/puppet/device/` 下定义或复用 provider 接口。
2. 在 `src/cpp/puppet/device/` 下实现新的 provider。
3. 让 provider 的 `nextFrame(...)` 只填充 `model::PrimitiveFrame`。
4. 在 device provider factory 中注册新的 `device.type`。
5. 在 `config/device/` 添加配置样例。
6. 通过 `device_service` 选择 `channel.type` 发布，不在 provider 中处理 protobuf 或 socket 细节。

边界要求：设备 SDK 相关逻辑留在 provider 内，通信封装留在 `transport`，不要把 app 启动逻辑塞进 provider。

### 6.2 新增 Runtime Channel

推荐接入路径：

1. 实现 `transport::RuntimeChannel` 派生类。
2. 增加对应 config 结构与 YAML loader。
3. 在 `PuppetConfig` 中接入新 channel 配置。
4. 在 `PuppetManager::createRuntimeChannel` 中按 `runtime_channel.type` 实例化。
5. 增加最小 sender/receiver demo 或脚本验证收发。

Runtime channel 应提供两类能力：接收 `PrimitiveFrame`，发布 `ControlIntent`。Robot state frame handler 也在 channel 层接入。

### 6.3 新增 Retargeting Plugin

推荐接入路径：

1. 实现 `retargeting::RetargetingPlugin` 接口。
2. 明确输入 primitive 类型、body group、control semantics 和输出的 `GroupControlIntent`。
3. 在 `RetargetingPipeline` 中注册 plugin type。
4. 给 `RuntimeConfig` / YAML loader 增加必要配置。
5. 在 `config/runtime/modules/.../pipelines.yaml` 中新增 pipeline。
6. 在 `group_routing.yaml` 中把 body group 指向新 pipeline。

如果 plugin 依赖 robot state，应通过 pipeline 的需求声明触发 `RobotStateSync` 合并，而不是自行跨层访问 channel。

### 6.4 新增 Robot Backend

当前 backend 入口是 `DirectMappingBackend`。新增 backend 时应保持输入为 `ControlIntent`，输出为明确的 final target 数据结构。backend 负责机器人执行语义映射，不应回头解析原始设备帧。

## 7. Demo 与验证

常用脚本：

```bash
./scripts/start_single_chain_ik_modular_embosa.sh
./scripts/start_single_chain_ik_modular_zmq.sh
./scripts/start_single_chain_ik_modular_device_service_embosa.sh
./scripts/start_single_chain_ik_modular_device_service_zmq.sh
./scripts/start_retargeting_3nodes.sh
./scripts/start_retargeting_3nodes_zmq.sh
```

常用 demo 目标：

- `test_embosa_puppet_proto_sender`
- `test_embosa_puppet_proto_receiver`
- `test_embosa_single_chain_ik_sender`
- `test_zmq_single_chain_ik_sender`
- `retargeting_mujoco_visualizer`
- `retargeting_mujoco_visualizer_zmq`

目前 `test/unit` 和 `test/integration` 尚未接入实际 `add_test` 用例。修改核心链路后，至少应完成：

- proto 构建通过。
- 主工程构建通过。
- 对应 Embosa 或 ZMQ demo 脚本能启动到预期阶段。
- 若改动涉及 visualizer，确认 MuJoCo/GLFW 依赖可用并运行对应工具。

## 8. 开发风格

- C++ 使用 C++17。
- 类名使用 `PascalCase`，函数使用 `camelCase`，局部变量使用 `snake_case`，成员变量使用 `trailingUnderscore_`。
- 对外头文件接口尽量短，并保持职责清晰。
- 配置优先使用显式 `Config` 结构体，不要把参数硬编码到算法实现中。
- 修改 C++ 后按仓库 `.clang-format` 格式化，可参考 `format_cpp.sh`。
- 更完整规则见仓库根目录 `AGENTS.md`。

## 9. 排查建议

- 构建找不到 `puppet_proto`：先执行 `./auto_build.sh --proto-only --install-proto`，或显式设置 `CMAKE_PREFIX_PATH`。
- 找不到 Embosa/ZMQ：检查 `ROOT_DEVEL_ARCH_PATH` 是否指向包含 `include/` 和 `lib/` 的安装前缀。
- 找不到 scaled device SDK：检查 `third_party/galbot_remote_operate/include` 和 `third_party/galbot_remote_operate/lib` 是否完整，或设置 `SCALED_DEVICE_SDK_ROOT`。
- ZMQ demo 无数据：确认 sender endpoint、runtime endpoint、viewer endpoint 与 YAML 配置一致，并检查端口是否被占用。
- Runtime 提示 stale robot state：检查对应 pipeline 是否需要 robot state，以及 robot state source 是否按 freshness timeout 正常发布。
