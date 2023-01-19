load("@rules_cc//cc:defs.bzl", "cc_library")
load("//third_party/gpus:common.bzl", "gpu_library", "if_cuda", "if_rocm")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

gpu_library(
    name = "perception_inference_operators_cuda",
    srcs = ["roipooling_layer.cu"],
    hdrs = ["roipooling_layer.h"],
    deps = [
        "//modules/perception/base:blob",
        "//modules/perception/inference:layer_lib",
        "@eigen",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

cc_library(
    name = "perception_inference_operators",
    srcs = [
        "roipooling_layer.cc",
        ":perception_inference_operators_cuda",
    ],
    hdrs = ["roipooling_layer.h"],
    deps = [
        "//modules/perception/base:blob",
        "//modules/perception/inference:layer_lib",
    ],
)

#cc_test(
#    name = "perception_inference_operators_test",
#    size = "small",
#    srcs = ["roipooling_layer_test.cc"],
#    deps = [
#        ":perception_inference_operators",
#        "@com_google_googletest//:gtest_main",
#    ],
#)

cpplint()
