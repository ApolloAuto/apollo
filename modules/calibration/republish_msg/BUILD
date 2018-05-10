load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "republish_msg_lib",
    srcs = ["republish_msg.cc"],
    hdrs = ["republish_msg.h"],
    deps = [
        "//modules/calibration/republish_msg/common:republish_msg_common",
        "//modules/calibration/republish_msg/proto:relative_odometry_proto",
        "//modules/common:apollo_app",
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/drivers/gnss/proto:ins_proto",
        "//modules/localization/proto:gps_proto",
    ],
)

cc_binary(
    name = "republish_msg",
    srcs = ["main.cc"],
    data = [
        "//modules/calibration/republish_msg/conf:republish_msg_adapter_manager_config",
        "//modules/calibration/republish_msg/conf:republish_msg_config",
    ],
    deps = [
        ":republish_msg_lib",
        "//external:gflags",
        "//modules/common:log",
        "@ros//:ros_common",
    ],
)

cpplint()
