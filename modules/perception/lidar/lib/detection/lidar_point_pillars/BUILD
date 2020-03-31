load("//tools:cpplint.bzl", "cpplint")
load("//tools:cuda_library.bzl", "cuda_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "point_pillars",
    srcs = [
        "src/point_pillars.cc",
        ":anchor_mask_cuda",
        ":nms_cuda",
        "postprocess_cuda",
        "scatter_cuda",
    ],
    hdrs = [
        "include/common.h",
        "include/anchor_mask_cuda.h",
        "include/nms_cuda.h",
        "include/point_pillars.h",
        "include/postprocess_cuda.h",
        "include/scatter_cuda.h",
    ],
    deps = [
        ":preprocess_points",
        "@cuda",
        "@tensorrt",
    ],
    alwayslink = True,
)

cc_library(
    name = "point_pillars_detection",
    srcs = [
        "src/point_pillars_detection.cc",
    ],
    hdrs = [
        "include/point_pillars_detection.h",
    ],
    deps = [
        ":point_pillars",
        "//cyber/common:common",
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
        "src/preprocess_points.cc",
        ":preprocess_points_cuda",
    ],
    hdrs = [
        "include/preprocess_points.h",
        "include/preprocess_points_cuda.h",
    ],
)

cuda_library(
    name = "anchor_mask_cuda",
    srcs = ["src/anchor_mask_cuda.cu"],
    hdrs = ["include/anchor_mask_cuda.h"],
    deps = ["@cuda"],
)

cuda_library(
    name = "nms_cuda",
    srcs = ["src/nms_cuda.cu"],
    hdrs = ["include/nms_cuda.h"],
    deps = ["@cuda"],
)

cuda_library(
    name = "postprocess_cuda",
    srcs = ["src/postprocess_cuda.cu"],
    hdrs = ["include/postprocess_cuda.h"],
    deps = [
        "@cuda",
    ],
)

cuda_library(
    name = "preprocess_points_cuda",
    srcs = ["src/preprocess_points_cuda.cu"],
    hdrs = ["include/preprocess_points_cuda.h"],
    deps = ["@cuda"],
)

cuda_library(
    name = "scatter_cuda",
    srcs = ["src/scatter_cuda.cu"],
    hdrs = ["include/scatter_cuda.h"],
    deps = ["@cuda"],
)

cc_test(
    name = "point_pillars_test",
    size = "small",
    srcs = ["test/src/test_point_pillars.cc"],
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