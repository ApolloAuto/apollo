load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "plugin_interface",
    srcs = [],
    hdrs = ["plugin_interface.h"],
    deps = [
        "//third_party/json",
    ],
)

cc_library(
    name = "metric_data_single_des_plugin",
    srcs = ["metric_data_single_plugin.cc"],
    hdrs = ["metric_data_single.h"],
    deps = [
        ":plugin_interface",
    ],
)

cc_test(
    name = "test_metric_data",
    size = "small",
    srcs = [
        "test_metric_data.cc",
    ],
    deps = [
        ":metric_data_single_des_plugin",
        "@gtest//:main",
    ],
)


cpplint()
