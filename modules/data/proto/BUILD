load("//tools:py_proto.bzl", "py_proto")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "static_info_proto",
    deps = [":static_info_proto_lib"],
)

proto_library(
    name = "static_info_proto_lib",
    srcs = ["static_info.proto"],
    deps = [
        "//modules/canbus/proto:canbus_proto_lib",
        "//modules/common/configs/proto:vehicle_config_proto_lib",
        "//modules/control/proto:control_proto_lib",
        "//modules/routing/proto:routing_proto_lib",
    ],
)

proto_library(
    name = "tools_frame_proto_lib",
    srcs = ["frame.proto"],
)

py_proto(
    name = "frame_pb2",
    src = "frame.proto",
)

py_proto(
    name = "static_info_pb2",
    src = "static_info.proto",
    deps = [
        "//modules/canbus/proto:canbus_conf_pb2",
        "//modules/canbus/proto:chassis_pb2",
        "//modules/common/configs/proto:vehicle_config_pb2",
        "//modules/control/proto:control_conf_pb2",
        "//modules/routing/proto:routing_pb2",
    ],
)
