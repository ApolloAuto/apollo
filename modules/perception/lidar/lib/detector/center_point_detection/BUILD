load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "center_point_detection",
    srcs = [
        "center_point_detection.cc",
    ],
    hdrs = [
        "center_point_detection.h",
    ],
    deps = [
        ":params",
        "//cyber/common",
        "//modules/perception/base",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/lib/thread",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface:base_lidar_detector",
        "//modules/perception/pipeline:stage",
        "@centerpoint_infer_op",
        "@eigen",
        "@local_config_pcl//:pcl",
        "@local_config_tensorrt//:tensorrt",
        "@paddleinference//:paddleinference_lib",
    ],
    alwayslink = True,
)

cc_library(
    name = "common",
    hdrs = ["common.h"],
)

cc_library(
    name = "params",
    hdrs = ["params.h"],
)

cpplint()
