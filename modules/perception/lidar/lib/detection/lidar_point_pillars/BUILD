load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "point_pillars",
    srcs = [
        "point_pillars.cc",
        ":anchor_mask_cuda",
        ":nms_cuda",
        ":postprocess_cuda",
        ":scatter_cuda",
    ],
    hdrs = [
        "anchor_mask_cuda.h",
        "common.h",
        "nms_cuda.h",
        "params.h",
        "point_pillars.h",
        "postprocess_cuda.h",
        "scatter_cuda.h",
    ],
    deps = [
        ":preprocess_points",
        "//modules/perception/common:perception_gflags",
        "@local_config_cuda//cuda:cudart",
        "@tensorrt",
    ],
    alwayslink = True,
)

cc_library(
    name = "point_pillars_detection",
    srcs = [
        "point_pillars_detection.cc",
    ],
    hdrs = [
        "point_pillars_detection.h",
    ],
    deps = [
        ":point_pillars",
        "//cyber/common",
        "//modules/perception/base",
        "//modules/perception/lib/thread",
        "//modules/perception/lidar/common",
        "@eigen",
    ],
    alwayslink = True,
)

cc_library(
    name = "preprocess_points",
    srcs = [
        "preprocess_points.cc",
        ":preprocess_points_cuda",
    ],
    hdrs = [
        "preprocess_points.h",
        "preprocess_points_cuda.h",
    ],
)

cuda_library(
    name = "anchor_mask_cuda",
    srcs = ["anchor_mask_cuda.cu"],
    hdrs = ["anchor_mask_cuda.h"],
    deps = ["@local_config_cuda//cuda:cudart"],
)

cuda_library(
    name = "nms_cuda",
    srcs = ["nms_cuda.cu"],
    hdrs = ["nms_cuda.h"],
    deps = ["@local_config_cuda//cuda:cudart"],
)

cuda_library(
    name = "postprocess_cuda",
    srcs = ["postprocess_cuda.cu"],
    hdrs = ["postprocess_cuda.h"],
    deps = [
        "@local_config_cuda//cuda:cudart",
    ],
)

cuda_library(
    name = "preprocess_points_cuda",
    srcs = ["preprocess_points_cuda.cu"],
    hdrs = ["preprocess_points_cuda.h"],
    deps = ["@local_config_cuda//cuda:cudart"],
)

cuda_library(
    name = "scatter_cuda",
    srcs = ["scatter_cuda.cu"],
    hdrs = ["scatter_cuda.h"],
    deps = ["@local_config_cuda//cuda:cudart"],
)

cc_test(
    name = "point_pillars_test",
    size = "small",
    srcs = ["point_pillars_test.cc"],
    deps = [
        ":point_pillars",
        ":preprocess_points",
        "//modules/perception/tool/benchmark/lidar/util:benchmark_util",
        "@boost",
        "@com_google_googletest//:gtest_main",
        "@eigen",
        "@pcl",
    ],
)

cpplint()
