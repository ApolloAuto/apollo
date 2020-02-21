load("//tools:cpplint.bzl", "cpplint")
load("//tools:cuda_library.bzl", "cuda_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cuda_library(
    name = "perception_inference_operators_cuda",
    srcs = ["roipooling_layer.cu"],
    hdrs = ["roipooling_layer.h"],
    deps = [
        "//modules/perception/base:blob",
        "//modules/perception/inference:layer_lib",
        "@cuda",
        "@eigen",
    ],
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
