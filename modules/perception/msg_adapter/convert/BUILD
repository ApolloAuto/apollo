load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "convert",
    hdrs = [
        "convert.h",
    ],
    deps = [
        ":common",
        "//cyber",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/perception/common/onboard/inner_component_messages:camera_detection_component_messages",
        "//modules/perception/common/onboard/inner_component_messages",
        "//modules/perception/common/onboard/inner_component_messages:lidar_inner_component_messages",
    ]
)

cc_library(
    name = "common",
    hdrs = [
        "common.h",
    ],
    deps = [
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/perception/common/base",
    ]
)

apollo_package()
cpplint()
