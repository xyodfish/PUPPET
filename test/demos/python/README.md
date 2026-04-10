# Proto Message Demo

## 1. 生成 Python protobuf 文件

在仓库根目录执行：

```bash
mkdir -p proto/python
protoc --proto_path=proto/proto --python_out=proto/python \
  proto/proto/puppet/common.proto \
  proto/proto/puppet/uniform.proto \
  proto/proto/puppet/primitive_frame.proto \
  proto/proto/puppet/control_intent.proto \
  proto/proto/puppet/primitive_trajectory.proto
```

## 2. 运行 demo

```bash
python3 test/demos/python/proto_message_demo.py \
  --proto-python-root proto/python \
  --out-dir /tmp/puppet_proto_demo
```

## 3. 输出内容

输出目录会生成：

- `primitive_frame.json` / `.pb`
- `control_intent.json` / `.pb`
- `primitive_trajectory.json` / `.pb`

这些文件分别演示：

1. 设备输入语义（`PrimitiveFrame`）
2. 控制意图语义（`ControlIntent`）
3. 轨迹输入语义（`PrimitiveFrameTrajectory`）
