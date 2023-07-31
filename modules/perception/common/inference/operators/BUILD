load("@rules_cc//cc:defs.bzl", "cc_library")
load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cuda_library(
    name = "perception_inference_operators_cuda",
    srcs = ["roipooling_layer.cu"],
    hdrs = ["roipooling_layer.h"],
    alwayslink = True,
    deps = [
        "//modules/perception/common/base:blob",
        "//modules/perception/common/inference:layer_lib",
        "@eigen",
        "@local_config_cuda//cuda:cudart",
    ],
)

cc_library(
    name = "perception_inference_operators",
    srcs = [
        "roipooling_layer.cc",
        ":perception_inference_operators_cuda",
    ],
    alwayslink = True,
    hdrs = ["roipooling_layer.h"],
    deps = [
        "//modules/perception/common/base:blob",
        "//modules/perception/common/inference:layer_lib",
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

apollo_package()
cpplint()
