load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "radar_obstacle_perception",
    srcs = ["radar_obstacle_perception.cc"],
    hdrs = ["radar_obstacle_perception.h"],
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:conti_radar_cc_proto",
        "//modules/common/util:util_tool",
        "//modules/perception/common:util",
        "//modules/perception/common/base",
        "//modules/perception/common/lib/registerer",
        "//modules/perception/radar_detection/interface:base_radar_obstacle_perception",
        "//modules/perception/radar_detection/proto:radar_obstacle_perception_cc_proto"
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
