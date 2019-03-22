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
