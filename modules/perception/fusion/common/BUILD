load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_filter",
    hdrs = ["base_filter.h"],
    deps = [
        "@eigen",
    ],
)

cc_library(
    name = "camera_util",
    srcs = ["camera_util.cc"],
    hdrs = ["camera_util.h"],
    deps = [
        "//modules/perception/base:camera",
        "//modules/perception/common/geometry:common",
        "//modules/perception/fusion/base:sensor",
    ],
)

cc_test(
    name = "camera_util_test",
    size = "small",
    srcs = ["camera_util_test.cc"],
    deps = [
        ":camera_util",
        "//modules/perception/fusion/base",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "dst_evidence",
    srcs = ["dst_evidence.cc"],
    hdrs = ["dst_evidence.h"],
    deps = [
        "//cyber",
    ],
)

cc_test(
    name = "dst_evidence_test",
    size = "small",
    srcs = ["dst_evidence_test.cc"],
    deps = [
        ":dst_evidence",
        "//modules/perception/fusion/base",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "information_filter",
    srcs = ["information_filter.cc"],
    hdrs = ["information_filter.h"],
    deps = [
        ":base_filter",
    ],
)

cc_test(
    name = "information_filter_test",
    size = "small",
    srcs = ["information_filter_test.cc"],
    deps = [
        ":information_filter",
        "//modules/perception/fusion/base",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "kalman_filter",
    srcs = ["kalman_filter.cc"],
    hdrs = ["kalman_filter.h"],
    deps = [
        ":base_filter",
        "//cyber",
    ],
)

cc_test(
    name = "kalman_filter_test",
    size = "small",
    srcs = ["kalman_filter_test.cc"],
    deps = [
        ":kalman_filter",
        "//modules/perception/fusion/base",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
