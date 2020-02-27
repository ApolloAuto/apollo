load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "smart_recorder_triggers_proto",
    deps = [":smart_recorder_triggers_proto_lib"],
)

proto_library(
    name = "smart_recorder_triggers_proto_lib",
    srcs = ["smart_recorder_triggers.proto"],
)

cc_proto_library(
    name = "smart_recorder_status_proto",
    deps = [":smart_recorder_status_proto_lib"],
)

proto_library(
    name = "smart_recorder_status_proto_lib",
    srcs = ["smart_recorder_status.proto"],
    deps = ["//modules/common/proto:header_proto_lib"],
)

py_proto(
    name = "smart_recorder_status_pb2",
    src = "smart_recorder_status.proto",
    deps = [
        "//modules/common/proto:header_pb2",
    ],
)

py_proto(
    name = "smart_recorder_triggers_pb2",
    src = "smart_recorder_triggers.proto",
)
