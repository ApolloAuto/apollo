load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "os_interface",
    srcs = ["os_interface.cc"],
    hdrs = ["os_interface.h"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/v2x/common:v2x_proxy_gflags",
        "//modules/v2x/proto:v2x_obstacles_cc_proto",
        "//modules/v2x/proto:v2x_obu_rsi_cc_proto",
        "//modules/v2x/proto:v2x_obu_traffic_light_cc_proto",
        "//modules/common_msgs/v2x_msgs:v2x_traffic_light_cc_proto",
    ],
)

cc_test(
    name = "os_interface_test",
    size = "small",
    timeout = "short",
    srcs = ["os_interface_test.cc"],
    deps = [
        ":os_interface",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cpplint()
