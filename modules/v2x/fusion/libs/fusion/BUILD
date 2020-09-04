load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "v2x_fusion_lib",
    srcs = [
        "fusion.cc",
    ],
    hdrs = [
        "fusion.h",
        "km.h",
        "test_tools.h",
    ],
    copts = ['-DMODULE_NAME=\\"v2x_fusion\\"'],
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/v2x/fusion/configs:v2x_configs_lib",
        "//modules/v2x/fusion/libs/common/base",
    ],
)

cc_test(
    name = "fusion_test",
    size = "small",
    srcs = ["fusion_test.cc"],
    deps = [
        ":v2x_fusion_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "km_test",
    size = "small",
    srcs = ["km_test.cc"],
    deps = [
        ":v2x_fusion_lib",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
