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
        "@gtest//:main",
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
        "@gtest//:main",
    ],
)

filegroup(
    name = "inference_test_data",
    srcs = glob(["inference_test_data/**"]),
)

cpplint()
