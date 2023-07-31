load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
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
        "//cyber/common",
        "//modules/perception/common:util",
        "//modules/perception/common/base",
        "//modules/perception/common/lib/thread",
        "//modules/perception/common/lidar/common",
        "//modules/perception/lidar_detection/detector/point_pillars_detection:point_pillars",
        "//modules/perception/lidar_detection/interface:base_lidar_detector",
        "//modules/perception/lidar_detection/detector/mask_pillars_detection/proto:model_param_cc_proto",
        "@eigen",
        "@local_config_pcl//:pcl",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
