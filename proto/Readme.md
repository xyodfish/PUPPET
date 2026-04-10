# Puppet Proto Build

## 1. Build C++ + Python protobuf

From repository root (recommended via `auto_build.sh`):

```bash
./auto_build.sh --proto-only
```

Or directly:

```bash
cmake -S proto -B proto/build
cmake --build proto/build -j"$(nproc)"
```

This build will:

1. Generate C++ protobuf files in `proto/build/generated/cpp/puppet/*.pb.{h,cc}`
2. Generate Python protobuf files in `proto/build/generated/python/puppet/*_pb2.py`
3. Sync generated files to:
   - `proto/src/puppet/*.pb.cc`
   - `proto/include/puppet/*.pb.h`
   - `proto/python/puppet/*_pb2.py`

## 2. Install (optional)

```bash
cmake --install proto/build --prefix /tmp/puppet_proto_install
```

Install output includes:

- C++ library: `lib/libpuppet_proto.so`
- C++ headers: `include/puppet/*.pb.h`
- Python modules: `lib/python3/site-packages/puppet/*_pb2.py`
- Raw `.proto` files: `share/puppet/proto/puppet/*.proto`
