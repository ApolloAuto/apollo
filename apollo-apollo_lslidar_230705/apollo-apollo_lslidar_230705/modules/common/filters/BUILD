load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

install(
    name = "install",
    library_dest = "common/lib",
    data_dest = "common",
    targets = [
        ":libfilters.so",
    ],
    visibility = ["//visibility:public"],
)


cc_binary(
    name = "libfilters.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":digital_filter_coefficients",
        ":digital_filter",
        ":mean_filter",
    ],
)

cc_library(
    name = "filters",
    srcs = ["libfilters.so"],
    hdrs = [
        "digital_filter_coefficients.h",
        "digital_filter.h",
        "mean_filter.h",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "digital_filter",
    srcs = ["digital_filter.cc"],
    hdrs = ["digital_filter.h"],
    alwayslink = True,
    deps = [
        "//cyber:cyber",
    ],
)

cc_library(
    name = "mean_filter",
    srcs = ["mean_filter.cc"],
    hdrs = ["mean_filter.h"],
    alwayslink = True,
    deps = [
        "//cyber:cyber",
    ],
)

cc_library(
    name = "digital_filter_coefficients",
    srcs = ["digital_filter_coefficients.cc"],
    hdrs = ["digital_filter_coefficients.h"],
    alwayslink = True,
    deps = [
        "//cyber:cyber",
    ],
)

cc_test(
    name = "digital_filter_test",
    size = "small",
    srcs = ["digital_filter_test.cc"],
    deps = [
        ":digital_filter",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_test(
    name = "mean_filter_test",
    size = "small",
    srcs = ["mean_filter_test.cc"],
    deps = [
        ":mean_filter",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_test(
    name = "digital_filter_coefficients_test",
    size = "small",
    srcs = ["digital_filter_coefficients_test.cc"],
    deps = [
        ":digital_filter_coefficients",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
