load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "fusion",
    srcs = ["fusion.cc"],
    hdrs = ["fusion.h"],
    copts = ['-DMODULE_NAME=\\"v2x_fusion\\"'],
    deps = [
        ":km",
        "//modules/v2x/fusion/configs:ft_config_manager",
        "//modules/v2x/fusion/libs/common:v2x_object",
        "@eigen",
    ],
)

cc_library(
    name = "km",
    hdrs = ["km.h"],
    deps = [
        "@eigen",
    ],
)

cc_library(
    name = "test_tools",
    hdrs = ["test_tools.h"],
    deps = [
        "//modules/v2x/fusion/libs/common:v2x_object",
    ],
)

cc_test(
    name = "fusion_test",
    size = "small",
    srcs = ["fusion_test.cc"],
    deps = [
        ":fusion",
        ":test_tools",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "km_test",
    size = "small",
    srcs = ["km_test.cc"],
    deps = [
        ":km",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
