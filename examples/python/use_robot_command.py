# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause-Clear

from google.protobuf import json_format

from qrb.contract.control.v1 import robot_pb2


def build_command_message() -> robot_pb2.RobotCommandMessage:
    message = robot_pb2.RobotCommandMessage()
    message.header.stamp.sec = 1720000000
    message.header.stamp.nanosec = 123456789
    message.header.frame_id = "base_link"
    message.header.source_id = "robot-alpha"
    message.header.stream_id = "motion_command"
    message.header.sequence = 1001
    message.header.correlation_id = "cmd-1001"

    message.command.type = robot_pb2.COMMAND_TYPE_MOTION
    message.command.motion.linear_mps = 0.8
    message.command.motion.angular_radps = 0.15
    message.command.motion.duration_sec = 2.5
    return message


def main() -> None:
    message = build_command_message()

    wire_bytes = message.SerializeToString()

    decoded = robot_pb2.RobotCommandMessage()
    decoded.ParseFromString(wire_bytes)

    json_text = json_format.MessageToJson(
        decoded,
        preserving_proto_field_name=True,
    )

    print(f"source_id: {decoded.header.source_id}")
    print(f"stream_id: {decoded.header.stream_id}")
    print(f"sequence: {decoded.header.sequence}")
    print(f"stamp: {decoded.header.stamp.sec}.{decoded.header.stamp.nanosec}")
    print(f"frame_id: {decoded.header.frame_id}")
    print(f"type: {decoded.command.type}")
    print(f"linear_mps: {decoded.command.motion.linear_mps}")
    print(f"wire_size: {len(wire_bytes)} bytes")
    print(f"json_size: {len(json_text)} bytes")


if __name__ == "__main__":
    main()
