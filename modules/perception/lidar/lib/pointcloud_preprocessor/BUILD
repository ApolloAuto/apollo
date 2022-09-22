load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pointcloud_preprocessor",
    srcs = ["pointcloud_preprocessor.cc"],
    hdrs = ["pointcloud_preprocessor.h"],
    deps = [
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/util",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/perception/base",
        "//modules/perception/lib/registerer",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface:base_pointcloud_preprocessor",
        "//modules/perception/pipeline/proto/stage:pointcloud_preprocessor_config_cc_proto",
        "//modules/perception/pipeline:stage",
        "@eigen",
    ],
    alwayslink = True,
)

cpplint()
