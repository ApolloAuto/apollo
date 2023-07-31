load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "inner_component_messages",
    hdrs = ["inner_component_messages.h"],
    alwayslink = True,
)

cc_library(
    name = "lidar_inner_component_messages",
    hdrs = [
        "lidar_inner_component_messages.h",
    ],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/perception/common/lidar/common",
        "//modules/perception/common/onboard/inner_component_messages",
    ],
)

cc_library(
    name = "traffic_inner_component_messages",
    hdrs = [
        "traffic_inner_component_messages.h",
    ],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/perception/common/camera/common:pose",
        "//modules/perception/common/camera/common:trafficlight_frame",
        "//modules/perception/common/onboard/inner_component_messages:inner_component_messages",
        "@boost",
    ],
)

cc_library(
    name = "camera_inner_component_messages",
    hdrs = [
        "camera_inner_component_messages.h",
    ],
    alwayslink = True,
    deps = [
        "@eigen",
        "//cyber",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/perception/common/camera/common:camera_frame",
    ],
)

cc_library(
    name = "camera_detection_component_messages",
    hdrs = [
        "camera_detection_component_messages.h",
    ],
    alwayslink = True,
    deps = [
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common:data_provider",
    ],
)

apollo_package()
cpplint()
