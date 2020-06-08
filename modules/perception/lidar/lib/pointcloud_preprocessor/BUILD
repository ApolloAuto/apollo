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
        "//modules/perception/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/common",
        "@eigen",
    ],
)

cpplint()
