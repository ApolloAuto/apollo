load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "layer_lib",
    srcs = ["layer.cc"],
    hdrs = ["layer.h"],
    deps = [
        "//modules/perception/base:blob",
    ],
)

cc_test(
    name = "layer_test",
    size = "small",
    srcs = ["layer_test.cc"],
    deps = [
        ":layer_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "inference_lib",
    srcs = ["inference.cc"],
    hdrs = ["inference.h"],
    deps = [
        "//modules/perception/base:blob",
    ],
)

cc_test(
    name = "inference_test",
    size = "small",
    srcs = ["inference_test.cc"],
    deps = [
        ":inference_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "inference_factory",
    srcs = ["inference_factory.cc"],
    hdrs = ["inference_factory.h"],
    deps = [
        ":inference_lib",
        "//modules/perception/inference/libtorch:torch_det",
        "//modules/perception/inference/libtorch:torch_net",
        "//modules/perception/inference/tensorrt:rt_net",
        "//modules/perception/inference/utils:inference_util_lib",
    ],
)

cc_test(
    name = "inference_factory_test",
    size = "small",
    srcs = ["inference_factory_test.cc"],
    deps = [
        ":inference_factory",
        "@com_google_googletest//:gtest_main",
    ],
)

filegroup(
    name = "inference_test_data",
    srcs = glob(["inference_test_data/**"]),
)

cpplint()
