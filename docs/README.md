# PUPPET Documentation

PUPPET 的文档按“快速上手、开发扩展、通信与 demo、设计背景”组织。主 README 保留项目展示与快速入口，这里作为更稳定的文档索引。

PUPPET documentation is organized around quick start, developer extension, communication demos, and design notes. The root README stays lightweight and lively; this page acts as the documentation hub.

## 中文文档

- [开发者指南](developer_guide.md)：模块分层、runtime 数据流、配置结构、构建依赖、扩展方式和排查建议。
- [Proto Overview](proto_overview.md)：`PrimitiveFrame`、`ControlIntent`、trajectory 等 proto 语义。
- [Runtime Channel ZMQ 接入说明](runtime_channel_zmq_接入说明.md)：ZMQ runtime channel、配置和 demo 说明。
- [Device Service 接入与迁移记录](device_service_三方库接入与源码迁移记录.md)：统一 device service、provider 和 scaled device SDK 接入。
- [Retargeting 3-Nodes Demo](retargeting_3nodes_demo.md)：runtime、sender、visualizer 三节点示例。
- [Runtime Dispatch Capability-first Design](runtime_dispatch_capability_first_design.md)：runtime dispatch 设计记录。
- [遥操作框架设计总结](遥操作框架设计总结_中文.md)：PUPPET 早期架构设计背景。

## English Documentation

- [Developer Guide](developer_guide_en.md): module boundaries, runtime data flow, configuration, build dependencies, extension points, and troubleshooting.
- [Proto Overview](proto_overview.md): proto-level semantics for `PrimitiveFrame`, `ControlIntent`, and trajectories.

## Quick Links

- Root README: [../README.md](../README.md)
- Proto build guide: [../proto/Readme.md](../proto/Readme.md)
- C++ Embosa demo: [../test/demos/cpp/README_embosa_proto_demo.md](../test/demos/cpp/README_embosa_proto_demo.md)
- Python proto demo: [../test/demos/python/README.md](../test/demos/python/README.md)

## Suggested Reading Order

1. Start with [../README.md](../README.md) for the project overview and quick commands.
2. Read [developer_guide.md](developer_guide.md) or [developer_guide_en.md](developer_guide_en.md) before changing core code.
3. Read [proto_overview.md](proto_overview.md) before editing proto/model conversion.
4. Read the ZMQ or Device Service notes before adding new transport or device providers.
