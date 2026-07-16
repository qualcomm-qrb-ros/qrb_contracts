// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include <iostream>
#include <string>

#include <google/protobuf/stubs/common.h>

#include "qrb/contract/control/v1/robot.pb.h"

int main() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  qrb::contract::control::v1::RobotCommandMessage message;
  qrb::contract::common::v1::Header* header = message.mutable_header();
  header->mutable_stamp()->set_sec(1720000000);
  header->mutable_stamp()->set_nanosec(123456789);
  header->set_frame_id("base_link");
  header->set_source_id("robot-alpha");
  header->set_stream_id("motion_command");
  header->set_sequence(1001);
  header->set_correlation_id("cmd-1001");

  qrb::contract::control::v1::RobotCommand* command = message.mutable_command();
  command->set_type(qrb::contract::control::v1::COMMAND_TYPE_MOTION);

  qrb::contract::control::v1::MotionCommand* motion = command->mutable_motion();
  motion->set_linear_mps(0.8);
  motion->set_angular_radps(0.15);
  motion->set_duration_sec(2.5);

  std::string wire_bytes;
  if (!message.SerializeToString(&wire_bytes)) {
    std::cerr << "failed to serialize RobotCommandMessage\n";
    return 1;
  }

  qrb::contract::control::v1::RobotCommandMessage decoded;
  if (!decoded.ParseFromString(wire_bytes)) {
    std::cerr << "failed to parse RobotCommandMessage\n";
    return 1;
  }

  std::cout << "source_id: " << decoded.header().source_id() << '\n';
  std::cout << "stream_id: " << decoded.header().stream_id() << '\n';
  std::cout << "sequence: " << decoded.header().sequence() << '\n';
  std::cout << "stamp: " << decoded.header().stamp().sec() << "."
            << decoded.header().stamp().nanosec() << '\n';
  std::cout << "frame_id: " << decoded.header().frame_id() << '\n';
  std::cout << "type: " << decoded.command().type() << '\n';
  std::cout << "linear_mps: " << decoded.command().motion().linear_mps() << '\n';
  std::cout << "wire_size: " << wire_bytes.size() << " bytes\n";

  google::protobuf::ShutdownProtobufLibrary();
  return 0;
}
