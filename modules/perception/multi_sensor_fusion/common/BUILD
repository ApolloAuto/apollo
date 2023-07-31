load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
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
        "//modules/perception/common/base:camera",
        "//modules/perception/common/algorithm/geometry:common",
        "//modules/perception/multi_sensor_fusion/base:sensor",
    ],
)

cc_test(
    name = "camera_util_test",
    size = "small",
    srcs = ["camera_util_test.cc"],
    deps = [
        ":camera_util",
        "//modules/perception/common/algorithm/sensor_manager",
        "//modules/perception/multi_sensor_fusion/base",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "dst_evidence",
    srcs = ["dst_evidence.cc"],
    hdrs = ["dst_evidence.h"],
    deps = [
        "//cyber",
        "@boost",
    ],
)

cc_test(
    name = "dst_evidence_test",
    size = "small",
    srcs = ["dst_evidence_test.cc"],
    deps = [
        ":dst_evidence",
        "//modules/perception/multi_sensor_fusion/base",
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
        "//modules/perception/multi_sensor_fusion/base",
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
    copts = ["-fno-access-control"],
    deps = [
        ":kalman_filter",
        "//modules/perception/multi_sensor_fusion/base",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_package()
cpplint()
