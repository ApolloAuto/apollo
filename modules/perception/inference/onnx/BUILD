load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//third_party/gpus:common.bzl", "if_cuda", "if_rocm")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "onnx_obstacle_detector",
    srcs = ["onnx_obstacle_detector.cc"],
    hdrs = [
        "onnx_obstacle_detector.h",
    ],
    deps = [
        "//modules/perception/common:perception_gflags",
        "//modules/perception/inference:inference_lib",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
        "@local_config_tensorrt//:tensorrt",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
        "@local_config_rocm//rocm:migraphx",
    ]),
)

cc_library(
    name = "libtorch_obstacle_detector",
    srcs = ["libtorch_obstacle_detector.cc"],
    hdrs = [
        "libtorch_obstacle_detector.h",
    ],
    deps = [
        "//cyber",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/inference:inference_lib",
        "@libtorch_gpu",
    ],
)

cpplint()
