load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
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
        "//modules/perception/common:util",
        "//modules/perception/common/base",
        "//modules/perception/common/inference:inference_factory",
        "//modules/perception/common/inference:inference_lib",
        "//modules/perception/common/inference:model_util",
        "//modules/perception/common/lib/thread",
        "//modules/perception/common/lidar/common",
        "//modules/perception/lidar_detection/interface:base_lidar_detector",
        "//modules/perception/lidar_detection/detector/center_point_detection/proto:model_param_cc_proto",
        "@eigen",
        "@local_config_pcl//:pcl",
        "@local_config_tensorrt//:tensorrt",
    ] + select({
        "@platforms//cpu:x86_64": ["@centerpoint_infer_op-x86_64//:centerpoint_infer_op"],
        "@platforms//cpu:aarch64": ["@centerpoint_infer_op-aarch64//:centerpoint_infer_op"]
    }) + select({
        "@platforms//cpu:x86_64": ["@paddleinference-x86_64//:paddleinference_lib"],
        "@platforms//cpu:aarch64": ["@paddleinference-aarch64//:paddleinference_lib"]
    }),
    alwayslink = True,
)

cc_library(
    name = "params",
    hdrs = ["params.h"],
)

apollo_package()
cpplint()
