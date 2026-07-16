# Robot Contracts

Protocol Buffers contracts for robot control, telemetry, geometry, and vision
messages. The repository includes source schemas, generated C++/Python bindings,
and small serialization examples.

## Contents

- `proto/qrb/contract/common/v1/`: shared `Header` and `Time` messages.
- `proto/qrb/contract/control/v1/`: robot command and status messages.
- `proto/qrb/contract/geometry/v1/`: `Pose2D` geometry primitive.
- `proto/qrb/contract/telemetry/v1/`: joint state telemetry.
- `proto/qrb/contract/vision/v1/`: raw, compressed, and shared-memory image payloads.
- `examples/`: C++, Python, and textproto examples.

Top-level messages carry `qrb.contract.common.v1.Header` for timestamp, frame,
stream identity, sequencing, and correlation metadata. Payload messages stay
header-free so they can be nested or reused without duplicating metadata.

## Requirements

Ubuntu packages:

```bash
apt-get install -y cmake ninja-build g++ protobuf-compiler libprotobuf-dev python3-protobuf
```

## Generate Code

C++:

```bash
mkdir -p generated/cpp
protoc \
  --proto_path=proto \
  --cpp_out=generated/cpp \
  $(find proto -name '*.proto' | sort)
```

Python:

```bash
mkdir -p generated/python
protoc \
  --proto_path=proto \
  --python_out=generated/python \
  $(find proto -name '*.proto' | sort)
```

Generated files are written to `generated/cpp/` and `generated/python/`.

## Run Examples

C++:

```bash
cmake -S examples/cpp -B build/examples/cpp -G Ninja
cmake --build build/examples/cpp --target robot_command_example
./build/examples/cpp/robot_command_example
```

Python:

```bash
PYTHONPATH="$PWD/generated/python${PYTHONPATH:+:$PYTHONPATH}" \
  python3 examples/python/use_robot_command.py
```

`examples/cpp/CMakeLists.txt` builds from generated C++ files under
`generated/cpp`; run the C++ generation command again after changing `.proto`
files.
