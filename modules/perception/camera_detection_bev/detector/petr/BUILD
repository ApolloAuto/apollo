load("@rules_cc//cc:defs.bzl", "cc_library")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "postprocess",
    srcs = ["postprocess.cc"],
    hdrs = ["postprocess.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/base",
    ]
)

cc_library(
    name = "preprocess",
    srcs = ["preprocess.cc"],
    hdrs = ["preprocess.h"],
    deps = [
        "//cyber",
        "@opencv//:imgproc",
    ]
)

cc_library(
    name = "bev_obstacle_detector",
    srcs = ["bev_obstacle_detector.cc"],
    hdrs = ["bev_obstacle_detector.h"],
    deps = [
        ":postprocess",
        ":preprocess",
        "//cyber",
        "//modules/perception/common:util",
        "//modules/perception/camera_detection_bev:camera_frame",
        "//modules/perception/camera_detection_bev/interface:base_obstacle_detector",
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common:util",
        "//modules/perception/camera_detection_bev/detector/petr/proto:model_param_cc_proto",
        "@eigen",
        "@opencv//:imgproc",
    ] + select({
        "@platforms//cpu:x86_64": ["@paddleinference-x86_64//:paddleinference_lib"],
        "@platforms//cpu:aarch64": ["@paddleinference-aarch64//:paddleinference_lib"]
    }),
)

apollo_package()
cpplint()
