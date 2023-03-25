load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//third_party/gpus:common.bzl", "gpu_library", "if_cuda", "if_rocm")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "params",
    hdrs = ["params.h"],
)

cc_library(
    name = "point_pillars",
    srcs = [
        "point_pillars.cc",
    ],
    hdrs = [
        "point_pillars.h",
    ],
    deps = [
        ":anchor_mask_cuda",
        ":common",
        ":nms_cuda",
        ":params",
        ":pfe_cuda",
        ":postprocess_cuda",
        ":preprocess_points",
        ":scatter_cuda",
        "//cyber",
        "//modules/perception/common:perception_gflags",
    ] + if_cuda([
        "@libtorch_gpu_cuda",
        "@local_config_cuda//cuda:cudart",
        "@local_config_tensorrt//:tensorrt",
    ])+ if_rocm([
        "@libtorch_gpu_rocm",
        "@local_config_rocm//rocm:hip",
        "@local_config_rocm//rocm:migraphx",
    ]),
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
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/lib/thread",
        "//modules/perception/lidar/common",
        "//modules/perception/lidar/lib/interface:base_lidar_detector",
        "//modules/perception/pipeline/proto/stage:point_pillars_detection_config_cc_proto",
        "//modules/perception/pipeline:stage",
        "@eigen",
        "@local_config_pcl//:pcl",
    ],
    alwayslink = True,
)

cc_library(
    name = "preprocess_points",
    srcs = [
        "preprocess_points.cc",
    ],
    hdrs = [
        "preprocess_points.h",
    ],
    deps = [
        ":common",
        ":preprocess_points_cuda",
    ],
)

cc_library(
    name = "common",
    hdrs = ["common.h"],
)

gpu_library(
    name = "anchor_mask_cuda",
    srcs = ["anchor_mask_cuda.cu"],
    hdrs = ["anchor_mask_cuda.h"],
    deps = [
        ":common",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

gpu_library(
    name = "nms_cuda",
    srcs = ["nms_cuda.cu"],
    hdrs = ["nms_cuda.h"],
    deps = [
        ":common",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

gpu_library(
    name = "pfe_cuda",
    srcs = ["pfe_cuda.cu"],
    hdrs = ["pfe_cuda.h"],
    deps = [
        ":common",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

gpu_library(
    name = "postprocess_cuda",
    srcs = ["postprocess_cuda.cu"],
    hdrs = ["postprocess_cuda.h"],
    deps = [
        ":common",
        ":nms_cuda",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
        "@local_config_rocm//rocm:rocm_headers",
    ]),
)

gpu_library(
    name = "preprocess_points_cuda",
    srcs = ["preprocess_points_cuda.cu"],
    hdrs = ["preprocess_points_cuda.h"],
    deps = [
        ":common",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

gpu_library(
    name = "scatter_cuda",
    srcs = ["scatter_cuda.cu"],
    hdrs = ["scatter_cuda.h"],
    deps = [
        ":common",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

cc_test(
    name = "point_pillars_test",
    size = "large",
    srcs = ["point_pillars_test.cc"],
    deps = [
        ":point_pillars",
        ":preprocess_points",
        "//modules/perception/tool/benchmark/lidar/util:benchmark_util",
        "@boost",
        "@com_google_googletest//:gtest_main",
        "@eigen",
        "@local_config_pcl//:pcl",
    ],
)

cpplint()
