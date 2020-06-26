load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools:cpplint.bzl", "cpplint")

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
    deps = [
        "@local_config_cuda//cuda:cublas",
        "@local_config_cuda//cuda:cudart",
        "@local_config_cuda//cuda:cudnn",
    ],
)

cuda_library(
    name = "perception_inference_tensorrt_plugins_slice_cuda",
    srcs = ["slice_plugin.cu"],
    hdrs = ["slice_plugin.h"],
    deps = [
        "//modules/perception/inference/tensorrt:rt_common",
        "@eigen",
        "@local_config_cuda//cuda:cudart",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cuda_library(
    name = "perception_inference_tensorrt_plugins_argmax_cuda",
    srcs = ["argmax_plugin.cu"],
    hdrs = ["argmax_plugin.h"],
    deps = [
        "//modules/perception/inference/tensorrt:rt_common",
        "@eigen",
        "@local_config_cuda//cuda:cudart",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cuda_library(
    name = "perception_inference_tensorrt_plugins_softmax_cuda",
    srcs = ["softmax_plugin.cu"],
    hdrs = ["softmax_plugin.h"],
    deps = [
        "//modules/perception/inference/tensorrt:rt_common",
        "@eigen",
        "@local_config_cuda//cuda:cudart",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cuda_library(
    name = "perception_inference_tensorrt_plugins_leakyrelu_cuda",
    srcs = ["leakyReLU_plugin.cu"],
    hdrs = ["leakyReLU_plugin.h"],
    deps = [
        "//modules/perception/inference/tensorrt:rt_common",
        "@eigen",
        "@local_config_cuda//cuda:cudart",
        "@local_config_cuda//cuda:cudnn",
        "@local_config_tensorrt//:tensorrt",
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
        "@local_config_cuda//cuda:cublas",
    ],
)

cpplint()
