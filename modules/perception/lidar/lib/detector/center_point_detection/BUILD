load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
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
        "//modules/perception/inference:inference_factory",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/lib/thread",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface:base_lidar_detector",
        "//modules/perception/pipeline:stage",
        "@eigen",
        "@local_config_pcl//:pcl",
        "@local_config_tensorrt//:tensorrt",
    ] + select({
        "@platforms//cpu:x86_64": ["@centerpoint_infer_op-x86_64//:centerpoint_infer_op"],
        "@platforms//cpu:aarch64": ["@centerpoint_infer_op-aarch64//:centerpoint_infer_op"],
    })+ select({
        "@platforms//cpu:x86_64": ["@paddleinference-x86_64//:paddleinference_lib"],
        "@platforms//cpu:aarch64": ["@paddleinference-aarch64//:paddleinference_lib"],
    }),
    
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
