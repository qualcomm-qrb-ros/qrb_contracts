cmake_minimum_required(VERSION 3.20)

project(robot_command_example LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Protobuf REQUIRED)

get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(PROTO_GEN_DIR "${REPO_ROOT}/generated/cpp")

file(GLOB_RECURSE PROTO_GEN_SRCS CONFIGURE_DEPENDS
  "${PROTO_GEN_DIR}/*.pb.cc"
)

if(NOT PROTO_GEN_SRCS)
  message(FATAL_ERROR
    "Generated C++ protobuf files are missing. Run the C++ generation command "
    "from the repository root before building this example."
  )
endif()

add_library(robot_protocol_cpp STATIC
  ${PROTO_GEN_SRCS}
)

target_include_directories(robot_protocol_cpp
  PUBLIC
    "${PROTO_GEN_DIR}"
    ${Protobuf_INCLUDE_DIRS}
)

target_link_libraries(robot_protocol_cpp
  PUBLIC
    ${Protobuf_LIBRARIES}
)

add_executable(robot_command_example
  use_robot_command.cpp
)

target_link_libraries(robot_command_example
  PRIVATE
    robot_protocol_cpp
)
