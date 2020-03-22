load("//tools:cpplint.bzl", "cpplint")
load("//tools:cuda_library.bzl", "cuda_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "perception_inference_tensorrt_plugins",
    srcs = [
        ":perception_inference_tensorrt_plugins_argmax_cuda",
        ":perception_inference_tensorrt_plugins_leakyrelu_cuda",
        ":perception_inference_tensorrt_plugins_slice_cuda",
        ":perception_inference_tensorrt_plugins_softmax_cuda",
    ],
    hdrs = glob([
        "*.h",
    ]),
    linkstatic = True,
)

cuda_library(
    name = "perception_inference_tensorrt_plugins_slice_cuda",
    srcs = ["slice_plugin.cu"],
    hdrs = ["slice_plugin.h"],
    deps = [
        "//modules/perception/inference/tensorrt:rt_common",
        "@cuda",
        "@eigen",
        "@tensorrt",
    ],
)

cuda_library(
    name = "perception_inference_tensorrt_plugins_argmax_cuda",
    srcs = ["argmax_plugin.cu"],
    hdrs = ["argmax_plugin.h"],
    deps = [
        "//modules/perception/inference/tensorrt:rt_common",
        "@cuda",
        "@eigen",
        "@tensorrt",
    ],
)

cuda_library(
    name = "perception_inference_tensorrt_plugins_softmax_cuda",
    srcs = ["softmax_plugin.cu"],
    hdrs = ["softmax_plugin.h"],
    deps = [
        "//modules/perception/inference/tensorrt:rt_common",
        "@cuda",
        "@eigen",
        "@tensorrt",
    ],
)

cuda_library(
    name = "perception_inference_tensorrt_plugins_leakyrelu_cuda",
    srcs = ["leakyReLU_plugin.cu"],
    hdrs = ["leakyReLU_plugin.h"],
    deps = [
        "//modules/perception/inference/tensorrt:rt_common",
        "@cuda",
        "@eigen",
        "@tensorrt",
    ],
)

cc_test(
    name = "slice_plugin_test",
    size = "small",
    srcs = ["slice_plugin_test.cc"],
    deps = [
        ":perception_inference_tensorrt_plugins",
        "//modules/perception/inference/tensorrt:rt_common",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "argmax_plugin_test",
    size = "small",
    srcs = ["argmax_plugin_test.cc"],
    deps = [
        ":perception_inference_tensorrt_plugins",
        "//modules/perception/inference/tensorrt:rt_common",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
