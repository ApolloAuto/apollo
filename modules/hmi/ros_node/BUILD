package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "ros_node_service",
    srcs = [
        "ros_node_service.cc",
        "ros_node_service.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/adapters:adapter_manager",
        "//modules/control/common",
        "//modules/control/proto:control_proto",
        "//modules/hmi/proto:ros_node_proto",
        "//third_party/ros:ros_common",
        "@com_github_google_protobuf//:protobuf",
    ],
)
