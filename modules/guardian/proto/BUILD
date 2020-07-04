load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "guardian_proto",
    deps = [
        ":guardian_proto_lib",
    ],
)

proto_library(
    name = "guardian_proto_lib",
    srcs = ["guardian.proto"],
    deps = [
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/common/proto:drive_state_proto_lib",
        "//modules/common/proto:error_code_proto_lib",
        "//modules/common/proto:geometry_proto_lib",
        "//modules/common/proto:header_proto_lib",
        "//modules/common/proto:vehicle_signal_proto_lib",
        "//modules/control/proto:control_command_proto_lib",
    ],
)

cc_proto_library(
    name = "guardian_conf_proto",
    deps = [
        ":guardian_conf_proto_lib",
    ],
)

proto_library(
    name = "guardian_conf_proto_lib",
    srcs = ["guardian_conf.proto"],
)

py_proto(
    name = "guardian_conf_pb2",
    src = "guardian_conf.proto",
)

py_proto(
    name = "guardian_pb2",
    src = "guardian.proto",
    deps = [
        "//modules/common/proto:header_pb2",
        "//modules/control/proto:control_cmd_pb2",
    ],
)
