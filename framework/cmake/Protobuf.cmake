find_package(Protobuf REQUIRED)

set(PROTOBUF_PROTOC_EXECUTABLE /usr/bin/protoc)
include_directories(${PROTOBUF_INCLUDE_DIR})
link_directories(${PROTOBUF_LIBRARY_DIR})
