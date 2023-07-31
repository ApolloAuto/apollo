load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pointcloud_preprocessor",
    srcs = ["pointcloud_preprocessor.cc"],
    hdrs = ["pointcloud_preprocessor.h"],
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/perception/common/base",
        "//modules/perception/common/lidar/common",
        "//modules/perception/pointcloud_preprocess/preprocessor/proto:pointcloud_preprocessor_config_cc_proto",
        "//modules/perception/pointcloud_preprocess/interface:base_pointcloud_preprocessor",
        "//modules/perception/common/lib/registerer:registerer",
        "//modules/perception/common:util",
        "@eigen",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
