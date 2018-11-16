cmake_minimum_required(VERSION 2.8.3)

find_package(Protobuf REQUIRED)

include_directories(${PROTOBUF_INCLUDE_DIRS})

set(protos_dir ${PROJECT_SOURCE_DIR}/src/protos)
message(STATUS "Proto Source Dir: ${protos_dir}")
file(GLOB proto_files "${protos_dir}/*.proto")
message(STATUS "Proto Source Files: ${proto_files}")

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${proto_files})

message(STATUS "Include directories for Google Protocol Buffers ${PROTOBUF_INCLUDE_DIRS}")

add_library(protos ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(protos ${PROTOBUF_LIBRARIES})
