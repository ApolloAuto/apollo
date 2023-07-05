load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mask_pillars_detection",
    srcs = [
        "mask_pillars_detection.cc",
    ],
    hdrs = [
        "mask_pillars_detection.h",
    ],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/lib/thread",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/detector/point_pillars_detection:point_pillars",
        "//modules/perception/lidar/lib/interface:base_lidar_detector",
        "//modules/perception/pipeline:stage",
        "@eigen",
        "@local_config_pcl//:pcl",
    ],
    alwayslink = True,
)

cpplint()
