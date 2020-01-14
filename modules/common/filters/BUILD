load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "digital_filter",
    srcs = ["digital_filter.cc"],
    hdrs = ["digital_filter.h"],
    deps = [
        ":digital_filter_coefficients",
        "//cyber/common:log",
    ],
)

cc_library(
    name = "mean_filter",
    srcs = ["mean_filter.cc"],
    hdrs = ["mean_filter.h"],
    deps = [
        "//cyber/common:log",
    ],
)

cc_library(
    name = "digital_filter_coefficients",
    srcs = ["digital_filter_coefficients.cc"],
    hdrs = ["digital_filter_coefficients.h"],
    deps = [
        "//cyber",
    ],
)

cc_test(
    name = "digital_filter_test",
    size = "small",
    srcs = ["digital_filter_test.cc"],
    deps = [
        ":digital_filter",
        ":digital_filter_coefficients",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "mean_filter_test",
    size = "small",
    srcs = ["mean_filter_test.cc"],
    deps = [
        ":mean_filter",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "digital_filter_coefficients_test",
    size = "small",
    srcs = ["digital_filter_coefficients_test.cc"],
    deps = [
        ":digital_filter_coefficients",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
