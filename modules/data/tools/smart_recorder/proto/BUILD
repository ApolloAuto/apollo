load("//tools:cpplint.bzl", "cpplint")

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

cpplint()
