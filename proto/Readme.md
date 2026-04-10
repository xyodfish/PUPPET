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

## 3. Use With `find_package`

After install, `puppet_proto` exports a CMake package config at:

```text
<install_prefix>/lib/cmake/puppet_proto/puppet_proto-config.cmake
```

In downstream project:

```cmake
find_package(puppet_proto CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE puppet_proto::puppet_proto)
```

Configure downstream build with:

```bash
cmake -S . -B build \
  -DCMAKE_PREFIX_PATH=/home/yuxia/Workspace/PUPPET/devel/x86_64-Linux-GNU-9.4.0
```
