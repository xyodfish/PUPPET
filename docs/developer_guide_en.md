# PUPPET Developer Guide

This guide is for developers who need to read, extend, or debug PUPPET. It focuses on the current module boundaries, runtime data flow, configuration layout, and common extension points in this repository.

## 1. Project Scope

PUPPET is a teleoperation runtime framework for multi-joint robots. Instead of binding every input device directly to a robot-specific target format, PUPPET first normalizes input into `PrimitiveFrame`, then routes it through runtime orchestration, retargeting pipelines, and robot backends to produce robot-side control semantics.

Core goals:

- Support heterogeneous input sources: VR, motion capture, master arms, file replay, and external messages.
- Represent device-side control semantics with `PrimitiveFrame`.
- Route ownership by source and body group.
- Support pluggable retargeting: direct pass-through, single-chain IK, GMR, and future plugins.
- Support multiple communication backends, currently Embosa and ZMQ.

## 2. Code Layout

```text
include/puppet/
  common/                 # Time, geometry, and common types
  primitive/              # PrimitiveFrame / PrimitiveTrajectory C++ models
  control/                # ControlIntent C++ models
  config/                 # Top-level PuppetConfig
  runtime/                # TeleopRuntime, RobotStateSync, RuntimeConfig
  source/                 # SourceManager
  orchestrator/           # Body-group routing and plan resolution
  retargeting/            # Plugin interface and retargeting implementations
  robot/backend/          # Mapping from ControlIntent to final target
  transport/              # Embosa/ZMQ runtime channels and device output channels
  device/                 # DeviceService and device provider interfaces

src/cpp/puppet/
  # Core implementations matching include/puppet/

app/cpp/
  runtime/                # Runtime executable entry points
  devices/                # Device service / replay entry points
  tools/                  # Visualizer and internal tools

config/
  runtime/                # Top-level and modular runtime YAML
  device/                 # DeviceService / replay / scaled device YAML
  tools/                  # Visualizer YAML
```

Layering rules:

- `device` collects or generates `PrimitiveFrame`; it should not depend on app or robot backend logic.
- `transport` owns communication and proto conversion.
- `runtime` orchestrates the flow but should not contain concrete retargeting algorithms.
- `retargeting` converts input primitives into group control intents.
- `app/cpp` wires configuration and startup only.

## 3. Runtime Data Flow

The main loop is driven by `PuppetManager`:

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

Important classes:

- `PuppetManager`: top-level lifecycle, config loading, channel creation, robot-state handler registration, and runtime loop.
- `TeleopRuntime`: configures sources, routing, and pipelines; converts latest input frames into `ControlIntent` during `runOnce`.
- `SourceManager`: stores latest `PrimitiveFrame` values by source.
- `Orchestrator`: resolves source ownership, body group routing, pipeline IDs, and backend IDs.
- `RetargetingPipeline`: dispatches work to retargeting plugins by `pipeline_id`.
- `DirectMappingBackend`: current entry point from `ControlIntent` to final target.

## 4. Configuration

Top-level runtime YAML files live under `config/runtime/`. Modular examples are split under `config/runtime/modules/demo_single_chain_ik/`:

- `sources.yaml`: source IDs, source types, topics, freshness, and history.
- `group_routing.yaml`: body group ownership, pipeline, backend, and control semantics.
- `pipelines.yaml`: pipeline IDs and plugin types.
- `backends.yaml`: backend IDs and backend types.
- `single_chain_ik.yaml` / `gmr_plugin.yaml`: retargeting plugin parameters.
- `embosa_runtime.yaml` / `zmq_runtime.yaml`: runtime channel parameters.
- `robot_state.yaml`: robot-state freshness settings.

The top-level `runtime_channel.type` selects the runtime transport:

```yaml
runtime_channel:
  type: zmq   # embosa | zmq
```

Device Service configs live under `config/device/`. Important fields:

- `device.type`: `static_file_replay`, `single_chain_ik_sender`, `scaled_device`, etc.
- `channel.type`: `embosa` or `zmq`.
- `loop_hz`, `source_id`, `frame_id`, topic, and endpoint fields.

## 5. Build and Dependencies

Recommended build flow:

```bash
./auto_build.sh --proto-only --install-proto
./auto_build.sh --main-only
```

Common CMake overrides:

```bash
cmake -S . -B build \
  -DROOT_DEVEL_ARCH_PATH=/opt/robot/devel/x86_64-Linux-GNU-9.4.0 \
  -DSCALED_DEVICE_SDK_ROOT=$PWD/third_party/galbot_remote_operate
cmake --build build -j"$(nproc)"
```

Main dependencies include `yaml-cpp`, `Eigen3`, `glog`, `pinocchio`, `qpOASES`, `orocos-kdl`, `trac_ik`, `puppet_proto`, Embosa, ZMQ, and the Galbot Scaled Device SDK. MuJoCo visualizer targets additionally require MuJoCo and GLFW.

`puppet_proto` is expected under the default prefix used by `./auto_build.sh --install-proto`:

```text
devel/<system-processor>-Linux-GNU-<compiler-version>
```

## 6. Extension Points

### 6.1 Add an Input Device

Recommended path:

1. Reuse or extend the provider interface under `include/puppet/device/`.
2. Implement the provider under `src/cpp/puppet/device/`.
3. Make `nextFrame(...)` fill only `model::PrimitiveFrame`.
4. Register the new `device.type` in the provider factory.
5. Add an example YAML under `config/device/`.
6. Publish through `device_service` and choose `channel.type` in configuration.

Device SDK logic belongs in the provider. Protobuf and socket details belong in `transport`.

### 6.2 Add a Runtime Channel

Recommended path:

1. Implement a `transport::RuntimeChannel` subclass.
2. Add a config struct and YAML loader.
3. Add the channel config to `PuppetConfig`.
4. Instantiate it in `PuppetManager::createRuntimeChannel`.
5. Add a minimal sender/receiver demo or script.

Runtime channels should receive `PrimitiveFrame`, publish `ControlIntent`, and connect robot-state frame callbacks when needed.

### 6.3 Add a Retargeting Plugin

Recommended path:

1. Implement the `retargeting::RetargetingPlugin` interface.
2. Define accepted primitive types, body groups, control semantics, and `GroupControlIntent` outputs.
3. Register the plugin type in `RetargetingPipeline`.
4. Add config fields and YAML loading if needed.
5. Add a pipeline entry in `config/runtime/modules/.../pipelines.yaml`.
6. Route a body group to the new pipeline in `group_routing.yaml`.

If a plugin needs robot state, declare that requirement through the pipeline path so `RobotStateSync` can merge the latest robot snapshot.

### 6.4 Add a Robot Backend

The current backend entry point is `DirectMappingBackend`. New backends should consume `ControlIntent` and produce a clear final target representation. Backend code should not parse raw device frames.

## 7. Demos and Validation

Common scripts:

```bash
./scripts/start_single_chain_ik_modular_embosa.sh
./scripts/start_single_chain_ik_modular_zmq.sh
./scripts/start_single_chain_ik_modular_device_service_embosa.sh
./scripts/start_single_chain_ik_modular_device_service_zmq.sh
./scripts/start_retargeting_3nodes.sh
./scripts/start_retargeting_3nodes_zmq.sh
```

Common demo targets:

- `test_embosa_puppet_proto_sender`
- `test_embosa_puppet_proto_receiver`
- `test_embosa_single_chain_ik_sender`
- `test_zmq_single_chain_ik_sender`
- `retargeting_mujoco_visualizer`
- `retargeting_mujoco_visualizer_zmq`

`test/unit` and `test/integration` are currently structural placeholders. For core changes, validate at least:

- proto build passes.
- main project build passes.
- the relevant Embosa or ZMQ demo starts as expected.
- visualizer changes are checked with MuJoCo/GLFW available.

## 8. Style

- C++17 is the default language level.
- Classes use `PascalCase`, functions use `camelCase`, local variables use `snake_case`, and member variables use `trailingUnderscore_`.
- Public headers should keep interfaces small and responsibilities clear.
- Prefer explicit `Config` structs over scattered hard-coded parameters.
- Format C++ changes with the repository `.clang-format`; `format_cpp.sh` may be used as a helper.
- See `AGENTS.md` for the full repository conventions.

## 9. Troubleshooting

- `puppet_proto` not found: run `./auto_build.sh --proto-only --install-proto`, or set `CMAKE_PREFIX_PATH`.
- Embosa/ZMQ not found: check that `ROOT_DEVEL_ARCH_PATH` points to a prefix containing `include/` and `lib/`.
- Scaled device SDK not found: check `third_party/galbot_remote_operate/include` and `third_party/galbot_remote_operate/lib`, or set `SCALED_DEVICE_SDK_ROOT`.
- ZMQ demo has no data: verify sender/runtime/viewer endpoints and make sure the port is not already bound.
- Runtime reports stale robot state: check whether the pipeline requires robot state and whether the robot-state source is publishing within the freshness timeout.
