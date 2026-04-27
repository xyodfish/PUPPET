# PUPPET 后续扩展路线与通信后端规划

本文记录 PUPPET 在现有 `Embosa / ZMQ / TCP / UDP` 通信后端基础上的后续扩展建议。目标不是简单堆叠协议数量，而是围绕遥操作 runtime 的稳定性、可复现性、分布式能力和机器人生态兼容性，稳步完善系统闭环。

---

## 1. 当前状态

PUPPET 当前已经具备以下通信与运行时基础：

```text
Embosa
ZMQ
TCP
UDP
```

它们分别覆盖了不同使用场景：

| 后端 | 主要用途 |
|---|---|
| Embosa | 公司/实机中间件链路 |
| ZMQ | 开源 demo、轻量 pub/sub、快速验证 |
| TCP | 点对点可靠流式传输 |
| UDP | 点对点低延迟数据包传输 |

当前通信基础已经足够支撑早期 demo 和实机验证。后续扩展应优先服务于系统稳定性、录制回放、分布式部署和生态接入。

---

## 2. 后续优先补齐的系统能力

在继续增加通信协议之前，建议优先补齐以下基础能力：

```text
1. RuntimeChannel 的统一 endpoint 配置
2. Channel stats / latency / dropped frame 统计
3. SourceManager freshness / timeout 管理
4. Orchestrator ownership / fallback / deadman 机制
5. Recorder / replay 闭环
6. 最小单元测试和集成测试
7. CMake optional dependency 管理
```

其中，**CMake optional dependency** 是当前非常重要的一步。PUPPET 不应让核心库强依赖 Embosa、ZMQ、Scaled Device SDK 等所有三方组件。建议增加如下编译选项：

```cmake
option(PUPPET_WITH_EMBOSA "Build Embosa transport" ON)
option(PUPPET_WITH_ZMQ "Build ZMQ transport" ON)
option(PUPPET_WITH_TCP "Build TCP transport" ON)
option(PUPPET_WITH_UDP "Build UDP transport" ON)
option(PUPPET_WITH_SCALED_DEVICE "Build Galbot scaled device provider" OFF)
option(PUPPET_WITH_ROS2 "Build ROS2 bridge" OFF)
option(PUPPET_WITH_ZENOH "Build Zenoh transport" OFF)
option(PUPPET_WITH_NATS "Build NATS control plane" OFF)
option(PUPPET_WITH_MCAP "Build MCAP recorder/replay" OFF)
```

推荐保持一个最小开源主链路：

```text
ZMQ + StaticFileReplay + TeleopRuntime + Visualizer
```

其他通信和设备 SDK 作为 optional plugin 接入。

---

## 3. 推荐新增通信/工具后端

### 3.1 Zenoh

Zenoh 是下一个最值得接入的通信后端。

它适合机器人遥操作和分布式机器人系统的原因：

```text
低开销 pub/sub
支持 discovery
适合边缘网络和跨机器通信
支持 query / storage
可以与 ROS2/DDS 生态桥接
适合机器人、IoT、边缘设备
```

PUPPET 中 Zenoh 可以承担：

```text
PrimitiveFrame pub/sub
ControlIntent pub/sub
RobotState pub/sub
RuntimeStatus pub/sub
远程调试
跨机器遥操作实验
```

建议新增：

```text
ZenohRuntimeChannel
ZenohDeviceOutputChannel
ZenohRobotStateChannel
```

示例配置：

```yaml
runtime_channel:
  type: zenoh
  session:
    mode: client      # peer | client
    connect: ["tcp/127.0.0.1:7447"]

  inputs:
    primitive_frame:
      key: "puppet/primitive_frame/**"
      queue_size: 200

    robot_state:
      key: "puppet/robot_state/**"
      queue_size: 50

  outputs:
    control_intent:
      key: "puppet/control_intent"
```

推荐 key 命名：

```text
puppet/{robot_id}/{source_id}/primitive_frame
puppet/{robot_id}/control_intent
puppet/{robot_id}/robot_state
puppet/{robot_id}/runtime_status
```

---

### 3.2 ROS2 Bridge

ROS2 Bridge 的目标不是让 PUPPET 核心依赖 ROS2，而是提供机器人生态兼容层。

推荐方式：

```text
PUPPET Core
  <-> ZMQ / Zenoh / TCP
  <-> puppet_ros2_bridge
  <-> ROS2 topics
```

优先做 bridge node，而不是直接把 ROS2 写进 core runtime。

ROS2 Bridge 可以支持：

```text
PrimitiveFrame <-> ROS2 message
ControlIntent  -> ROS2 message
RobotState     <- sensor_msgs/JointState / TF / custom state
FinalTarget    -> robot controller topic
```

建议顺序：

```text
1. puppet_ros2_bridge
2. ROS2 custom messages 或 proto bytes topic
3. JointState / TF bridge
4. 后续再考虑 DirectDDSRuntimeChannel
```

---

### 3.3 NATS

NATS 更适合做控制面，而不是高频机器人控制主链路。

适合承载：

```text
runtime status
command / service request
configuration update
start / stop / mode switch
remote monitor
cloud / multi-machine coordination
```

建议先实现：

```text
NatsControlPlane
```

而不是 `NatsRuntimeChannel`。

示例配置：

```yaml
control_plane:
  type: nats
  url: "nats://127.0.0.1:4222"
  subjects:
    runtime_status: "puppet.runtime.status"
    command: "puppet.runtime.command"
    query: "puppet.runtime.query"
```

适合的命令：

```text
start_runtime
stop_runtime
switch_mode
start_recording
stop_recording
query_source_state
query_current_owner
```

---

### 3.4 MCAP Recorder / Replay

MCAP 不是通信协议，但它对 PUPPET 非常关键，建议作为 P1 能力加入。

PUPPET 应支持：

```text
McapRecorder
McapReplayDeviceProvider
McapTrajectorySampler
```

建议记录以下 channel：

```text
/puppet/device_raw_frame
/puppet/primitive_frame
/puppet/control_intent
/puppet/final_target
/puppet/robot_state
/puppet/runtime_status
/puppet/solver_debug
```

MCAP 会给 PUPPET 带来：

```text
可复现
可回放
可 debug
可 benchmark
可展示
```

它比继续增加一个普通通信协议更能提升项目价值。

---

### 3.5 Shared Memory / Iceoryx

Shared Memory 后端适合本机高频、大数据场景，例如：

```text
vision landmarks
depth image
point cloud
full body skeleton stream
high-rate robot state
large trajectory batch
```

可选方案：

```text
iceoryx
boost::interprocess
mmap ring buffer
POSIX shm
```

当前 PUPPET 主要传输 protobuf 小消息，ZMQ/TCP/Zenoh 已经足够。Shared Memory 可后期再做。

---

### 3.6 WebSocket / HTTP Gateway

WebSocket / HTTP 更适合做 dashboard 和调试界面，而不是主控制链路。

推荐作为 gateway：

```text
PUPPET Runtime
  -> RuntimeStatus
  -> WebSocketGateway
  -> Browser UI
```

可提供接口：

```text
GET /status
GET /sources
GET /owners
POST /command/start_record
POST /command/stop_record
WS  /stream/runtime_status
```

适合用于：

```text
网页 dashboard
远程可视化
浏览器控制台
调试面板
状态监控
```

---

### 3.7 MQTT

MQTT 更适合 IoT 和低频状态，不建议作为高频机器人控制主链路。

可用于：

```text
battery
device online/offline
remote status
simple command
```

优先级低于 Zenoh、MCAP、ROS2 Bridge、NATS。

---

### 3.8 gRPC

gRPC 适合 service API，不适合作为高频实时 streaming 控制主链路。

如果接入，建议提供：

```text
PuppetService
  GetRuntimeStatus
  SetMode
  StartRecording
  StopRecording
  UploadPrimitiveTrajectory
  QuerySourceState
```

它更像管理接口，而不是 runtime data plane。

---

## 4. 通信/工具后端优先级

| 优先级 | 通信/工具 | 建议用途 | 建议 |
|---|---|---|---|
| P0 | ZMQ | 默认开源 demo 主链路 | 已有，继续打磨 |
| P0 | TCP / UDP | 基础点对点、调试、低依赖 | 已有，继续稳定 |
| P0 | Embosa | 公司/实机链路 | 已有，但应 optional |
| P1 | Zenoh | 分布式 pub/sub、跨网络、ROS2 bridge | 强烈建议 |
| P1 | MCAP | 记录、回放、benchmark | 强烈建议 |
| P1 | ROS2 Bridge | 接 ROS 生态 | 强烈建议 |
| P2 | NATS | 控制面、状态、request/reply | 建议 |
| P2 | WebSocket / HTTP | dashboard / debug UI | 建议 |
| P3 | Shared Memory | 本机大数据高频传输 | 后期再做 |
| P3 | MQTT | IoT / 低频云端状态 | 可选 |
| P3 | gRPC | service API | 可选 |

---

## 5. 建议版本路线

### v0.1：稳定当前链路

```text
ZMQ runtime channel
DeviceService
StaticFileReplay
SingleChainIK
MuJoCo visualizer
README + docs
optional CMake dependency
```

### v0.2：补记录回放

```text
PrimitiveFrameTrajectory sampler
MCAP recorder
MCAP replay provider
runtime status logging
latency statistics
```

### v0.3：补分布式通信

```text
ZenohRuntimeChannel
ZenohDeviceOutputChannel
Zenoh bridge demo
```

### v0.4：补 ROS2 生态

```text
puppet_ros2_bridge
sensor_msgs/JointState -> RobotState PrimitiveFrame
ControlIntent -> custom ROS2 msg 或 proto bytes
TF frame bridge
```

### v0.5：补控制面

```text
NatsControlPlane
WebSocket dashboard
runtime command API
source ownership monitor
```

---

## 6. 通信抽象升级建议

当前已有 `IRuntimeChannel` 和 `IDeviceOutputChannel`。后续可以继续拆成更明确的能力接口：

```cpp
class IPrimitiveFrameInput {
public:
    virtual ~IPrimitiveFrameInput() = default;
    virtual bool tryPopPrimitiveFrame(model::PrimitiveFrame& frame) = 0;
};

class IControlIntentOutput {
public:
    virtual ~IControlIntentOutput() = default;
    virtual bool publishControlIntent(
        const model::ControlIntent& intent,
        std::string& error) = 0;
};

class IRobotStateInput {
public:
    virtual ~IRobotStateInput() = default;
    virtual bool tryPopRobotState(model::PrimitiveFrame& frame) = 0;
};
```

然后 `IRuntimeChannel` 可以组合这些能力：

```cpp
class IRuntimeChannel :
    public IPrimitiveFrameInput,
    public IControlIntentOutput,
    public IRobotStateInput {
public:
    virtual ~IRuntimeChannel() = default;
};
```

这样以后接入 Zenoh、ROS2、NATS、MCAP replay 会更清晰。

---

## 7. 最推荐的下一步

最推荐下一步添加这三个能力：

```text
1. MCAP recorder / replay
2. ZenohRuntimeChannel
3. ROS2 Bridge
```

推荐顺序：

```text
先 MCAP，再 Zenoh，再 ROS2 Bridge。
```

原因：

- **MCAP** 解决可复现、可回放、可 benchmark，是长期开发的地基。
- **Zenoh** 解决分布式机器人通信，比继续手写 TCP/UDP 更有框架感。
- **ROS2 Bridge** 解决机器人生态兼容，让 PUPPET 更容易接入仿真、可视化和实机控制栈。

最终通信体系建议保持为：

```text
ZMQ: 最小开源 demo
Embosa: 公司/实机链路
TCP/UDP: 低依赖基础通道
Zenoh: 分布式机器人通信
ROS2 Bridge: 生态兼容
NATS/WebSocket: 控制面和调试面
MCAP: 记录回放闭环
```

这比盲目增加通信协议更稳，也更符合 PUPPET 作为机器人遥操作 runtime framework 的定位。
