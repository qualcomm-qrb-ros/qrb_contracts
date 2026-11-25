# qrb_contracts

Shared interface contracts for the QRB (Qualcomm Robotics) platform.
This repository defines the C++ buffer descriptors and Protocol Buffer schemas
used to describe robotics sensor data across nodes and services.

## Overview

`qrb_contracts` provides two layers of contracts:

| Layer | Location | Purpose |
|---|---|---|
| **C++ Descriptors** | `include/` | In-process / DMA buffer metadata, used between QCNodes |
| **Protobuf Schemas** | `proto/` | Serialization schemas for cross-machine transport (e.g. cloud upload) |

Both layers mirror [ROS 2](https://docs.ros.org/en/rolling/) message definitions
(`geometry_msgs`, `nav_msgs`, `sensor_msgs`, `vision_msgs`) to allow zero-copy or low-overhead
mapping between the two representations.

## Directory Structure

```
qrb_contracts/
├── include/
│   └── QRB/Memory/
│       ├── Common/
│       │   └── Types.hpp                      # Shared geometry & sensor value types
│       ├── Geometry/
│       │   └── PoseDescriptor.hpp             # PoseDescriptor
│       ├── Navigation/
│       │   └── OdometryDescriptor.hpp         # OdometryDescriptor
│       ├── Sensor/
│       │   ├── ImuDescriptor.hpp              # ImuDescriptor
│       │   ├── JointStateBufferDescriptor.hpp # JointState_t & JointStateBufferDescriptor_t
│       │   └── PointCloudDescriptor.hpp       # PointCloudDescriptor
│       └── Vision/
│           ├── ObjectDetectionDescriptor.hpp    # ObjectDetectionDescriptor (common base)
│           ├── ObjectDetection2dDescriptor.hpp  # ObjectDetection2dDescriptor
│           └── ObjectDetection3dDescriptor.hpp  # ObjectDetection3dDescriptor
├── proto/
│   └── qrb/contract/
│       ├── geometry/v1/
│       │   ├── point.proto
│       │   ├── vector3.proto
│       │   ├── quaternion.proto
│       │   ├── pose.proto
│       │   ├── pose_with_covariance.proto
│       │   ├── pose_with_covariance_stamped.proto
│       │   ├── transform.proto
│       │   ├── transform_stamped.proto
│       │   ├── twist.proto
│       │   ├── twist_with_covariance.proto
│       │   └── twist_stamped.proto
│       ├── common/v1/
│       │   ├── time.proto
│       │   └── header.proto
│       ├── nav/v1/
│       │   └── odometry.proto
│       ├── tf2/v1/
│       │   └── tf_message.proto
│       ├── sensor/v1/
│       │   ├── imu.proto
│       │   ├── joint_state.proto
│       │   ├── point_field.proto
│       │   └── point_cloud2.proto
│       └── vision/v1/
│           ├── object_hypothesis.proto
│           ├── object_hypothesis_with_pose.proto
│           ├── bounding_box_3d.proto
│           ├── detection_3d.proto
│           └── detection_3d_array.proto

```

## C++ Descriptors

All descriptors inherit from `QC::Memory::BufferDescriptor_t` (defined in `qcnode`)
and live in the `QRB::Memory` namespace.

### Inheritance

```
QC::Memory::QCBufferDescriptorBase_t
└── QC::Memory::BufferDescriptor_t
    ├── QRB::Memory::PoseDescriptor_t          (Geometry/PoseDescriptor.hpp)
    ├── QRB::Memory::OdometryDescriptor_t      (Navigation/OdometryDescriptor.hpp)
    ├── QRB::Memory::ImuDescriptor_t           (Sensor/ImuDescriptor.hpp)
    ├── QRB::Memory::JointStateBufferDescriptor_t (Sensor/JointStateBufferDescriptor.hpp)
    ├── QRB::Memory::PointCloudDescriptor_t    (Sensor/PointCloudDescriptor.hpp)
    └── QRB::Memory::ObjectDetectionDescriptor_t      (Vision/ObjectDetectionDescriptor.hpp, abstract)
        ├── QRB::Memory::ObjectDetection2dDescriptor_t  (Vision/ObjectDetection2dDescriptor.hpp)
        └── QRB::Memory::ObjectDetection3dDescriptor_t  (Vision/ObjectDetection3dDescriptor.hpp)
```

### Common Types (`Common/Types.hpp`)

Shared geometry value types used across descriptors, mirroring `geometry_msgs`:
`Point3d_t`, `Vector3d_t`, `Quaternion_t`, `Pose_t`, `Transform_t`,
`PoseWithCovariance_t`, `Twist_t`, `TwistWithCovariance_t`, `BoundingBox3d_t`,
and, mirroring `vision_msgs`, the 2-D pixel types `Point2d_t`, `Pose2d_t`,
`BoundingBox2d_t`.

Types used by a single descriptor live in that descriptor's own header, as in
qcnode's `ImageDescriptor.hpp`. For example `PointFieldType_e` and `PointField_t`
(mirroring `sensor_msgs/PointField`) are in `Sensor/PointCloudDescriptor.hpp`.

### Descriptors

#### `PoseDescriptor_t`
Mirrors `geometry_msgs/PoseWithCovarianceStamped`. Carries a single stamped
6-DOF pose with a 6×6 covariance matrix.

#### `OdometryDescriptor_t`
Mirrors `nav_msgs/Odometry`. Carries the estimated pose (in `frameId`) and
velocity (in `childFrameId`), each with a 6×6 covariance matrix.

#### `ImuDescriptor_t`
Mirrors `sensor_msgs/Imu`. Carries a single IMU sample: the orientation
quaternion, the angular velocity, and the linear acceleration. Each of the
last three has a row-major 3×3 covariance matrix. Following the ROS convention, `covariance[0] == -1` means covariance is unknown.

#### `JointStateBufferDescriptor_t`
Mirrors `sensor_msgs/JointState`. Metadata is `timestampNs`, `frameId` and
`jointCount`; the host-owned payload at `pBuf + offset` is an array of
`JointState_t` records with a 63-byte-capacity name and `double` position,
velocity and effort. All measurements are present; zero does not mean missing.
Units are rad, rad/s, N·m for revolute joints and m, m/s, N for prismatic joints.

Construct records in aligned storage, allocate enough bytes for the offset
and `CalcBufferSize(jointCount)`, and set `validSize` to that payload size.
`GetJointPtr()` checks index, full-frame bounds and alignment; copies retain
buffer references, not payload ownership. Check fixed-string assignment status
and agree on the timestamp clock domain. Identify the descriptor by C++ type,
not the shared `QC_BUFFER_TYPE_CUSTOM_4` tag.

The matching `sensor/v1/joint_state.proto` preserves ROS parallel arrays and
independently missing measurements. `qrb_ros_adapter` supplies both conversion
directions; conversion to HCM requires all three arrays for nonempty frames
and rejects missing measurements rather than filling zeros.

#### `PointCloudDescriptor_t`
Mirrors `sensor_msgs/PointCloud2`. The raw point data is stored in the DMA
buffer (`pBuf`). Field layout is described by a fixed-size array of up to
`QRB_POINT_CLOUD_MAX_FIELDS` (16) `PointField_t` entries plus `numFields`,
allowing arbitrary per-point field combinations (XYZ, XYZI, XYZRGB, etc.).
Field names and `frameId` are fixed-capacity `QC::QCString`s, so the
descriptor has no dynamic-length member and copying it never allocates, as
qcnode requires of buffer descriptors.

#### `ObjectDetectionDescriptor_t`
Common, abstract base of the object detection descriptors. It holds what the 2-D
and 3-D descriptors share: the buffer type tag, `timestampNs`, `frameId`,
`objCount`, the assignment logic, and the shared record types
(`ObjectHypothesisRaw_t`, and `DetectionRaw<BoundingBoxT>`, which both
`Detection2dRaw_t` and `Detection3dRaw_t` are instances of). Instantiate one of
the concrete descriptors below.

All detection descriptors share the buffer type tag `QC_BUFFER_TYPE_CUSTOM_2`, so
tell them apart by C++ type (`dynamic_cast` to the concrete descriptor), not by
`type`. A `dynamic_cast` to `ObjectDetectionDescriptor_t` matches both.

Assigning a descriptor of the other dimension, or a plain `BufferDescriptor_t`,
copies the buffer fields only and leaves `timestampNs`, `frameId` and
`objCount` untouched, so a buffer of 2-D records is never labelled as 3-D.

The following limits are configurable via macros before including the header:

| Macro | Default | Description |
|---|---|---|
| `QRB_DETECTION_MAX_CLASS_ID_LEN` | `64` | Max class ID string length (bytes) |
| `QRB_DETECTION_MAX_ID_LEN` | `64` | Max tracking ID string length (bytes) |
| `QRB_DETECTION_MAX_RESULTS` | `8` | Max hypothesis results per detection |

#### `ObjectDetection2dDescriptor_t`
Mirrors `vision_msgs/Detection2DArray`. Detection results are stored as a
contiguous array of `Detection2dRaw_t` POD structs directly in the DMA buffer.
`objCount` records the number of valid entries. The bounding box is a
`BoundingBox2d_t` in pixels.

Buffer layout:
```
pBuf → [ Detection2dRaw_t[0] | Detection2dRaw_t[1] | ... | Detection2dRaw_t[objCount-1] ]
```

#### `ObjectDetection3dDescriptor_t`
Mirrors `vision_msgs/Detection3DArray`. Same as the 2-D descriptor, with
`Detection3dRaw_t` records whose bounding box is a `BoundingBox3d_t` in metres.

Buffer layout:
```
pBuf → [ Detection3dRaw_t[0] | Detection3dRaw_t[1] | ... | Detection3dRaw_t[objCount-1] ]
```

## Protobuf Schemas

Schemas are organised under `proto/qrb/contract/` with versioned sub-paths
(`v1/`) to allow non-breaking evolution. Each ROS 2 message type maps to one
`.proto` file. The directory name is the ROS 2 package name without the `_msgs`
suffix (`geometry_msgs` → `geometry`, `nav_msgs` → `nav`, `tf2_msgs` → `tf2`,
...) and the message name is identical to the ROS 2 one. Field names and
structure follow ROS 2 too, e.g. stamped messages carry a `std_msgs/Header`
(`header.stamp` is a `builtin_interfaces/Time` with `sec` / `nanosec`). The
basic types that most other messages build on, `std_msgs/Header` and
`builtin_interfaces/Time`, live in `common/` rather than in a package of their
own, which also avoids a `std` namespace shadowing `::std` in the generated C++
code.

### ROS 2 ↔ Proto mapping

| ROS 2 message | Proto file |
|---|---|
| `builtin_interfaces/Time` | `common/v1/time.proto` |
| `std_msgs/Header` | `common/v1/header.proto` |
| `geometry_msgs/Point` | `geometry/v1/point.proto` |
| `geometry_msgs/Vector3` | `geometry/v1/vector3.proto` |
| `geometry_msgs/Quaternion` | `geometry/v1/quaternion.proto` |
| `geometry_msgs/Pose` | `geometry/v1/pose.proto` |
| `geometry_msgs/PoseWithCovariance` | `geometry/v1/pose_with_covariance.proto` |
| `geometry_msgs/PoseWithCovarianceStamped` | `geometry/v1/pose_with_covariance_stamped.proto` |
| `geometry_msgs/Transform` | `geometry/v1/transform.proto` |
| `geometry_msgs/TransformStamped` | `geometry/v1/transform_stamped.proto` |
| `tf2_msgs/TFMessage` | `tf2/v1/tf_message.proto` |
| `geometry_msgs/Twist` | `geometry/v1/twist.proto` |
| `geometry_msgs/TwistWithCovariance` | `geometry/v1/twist_with_covariance.proto` |
| `geometry_msgs/TwistStamped` | `geometry/v1/twist_stamped.proto` |
| `nav_msgs/Odometry` | `nav/v1/odometry.proto` |
| `sensor_msgs/PointField` | `sensor/v1/point_field.proto` |
| `sensor_msgs/Imu` | `sensor/v1/imu.proto` |
| `sensor_msgs/JointState` | `sensor/v1/joint_state.proto` |
| `sensor_msgs/PointCloud2` | `sensor/v1/point_cloud2.proto` |
| `vision_msgs/ObjectHypothesis` | `vision/v1/object_hypothesis.proto` |
| `vision_msgs/ObjectHypothesisWithPose` | `vision/v1/object_hypothesis_with_pose.proto` |
| `vision_msgs/Point2D` | `vision/v1/point_2d.proto` |
| `vision_msgs/Pose2D` | `vision/v1/pose_2d.proto` |
| `vision_msgs/BoundingBox2D` | `vision/v1/bounding_box_2d.proto` |
| `vision_msgs/BoundingBox3D` | `vision/v1/bounding_box_3d.proto` |
| `vision_msgs/Detection2D` | `vision/v1/detection_2d.proto` |
| `vision_msgs/Detection2DArray` | `vision/v1/detection_2d_array.proto` |
| `vision_msgs/Detection3D` | `vision/v1/detection_3d.proto` |
| `vision_msgs/Detection3DArray` | `vision/v1/detection_3d_array.proto` |

## Dependencies

- **qcnode** — provides `QC::Memory::BufferDescriptor_t` and related base types
- **Protocol Buffers** (`protoc` ≥ 3.15) — required to generate code from `.proto` files

## License

Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.  
SPDX-License-Identifier: BSD-3-Clause-Clear
