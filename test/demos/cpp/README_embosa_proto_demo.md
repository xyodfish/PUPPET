# Embosa Proto Communication Demo

This demo contains two embosa nodes:

1. `test_embosa_puppet_proto_sender`
2. `test_embosa_puppet_proto_receiver`

Topic:

- `puppet_demo/primitive_frame`
- message type: `puppet::puppet_proto::PrimitiveFrame`

Behavior:

- sender publishes one `PrimitiveFrame` every 1 second.
- receiver parses protobuf into `puppet::model::PrimitiveFrame` using
  `puppet::transport::copyFromProto(...)` and prints key fields.

## Build

1. Build and install proto package first:

```bash
./auto_build.sh --proto-only --install-proto
```

2. Build project demos:

```bash
cmake -S . -B build
cmake --build build -j"$(nproc)"
```

Optional: if embosa is not in the default path (`/opt/galbot/devel/<arch-prefix>`), pass:

```bash
cmake -S . -B build -DROOT_DEVEL_ARCH_PATH=/opt/galbot/devel/x86_64-Linux-GNU-9.4.0
```

## Run

Terminal 1:

```bash
./build/test/demos/cpp/test_embosa_puppet_proto_receiver
```

Terminal 2:

```bash
./build/test/demos/cpp/test_embosa_puppet_proto_sender
```

Expected output:

- sender prints sequence id and sent pose x value once per second.
- receiver prints parsed `sourceId/mode/sequenceId` and first pose/scalar values.
