load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "point_cloud_processing",
    hdrs = [
        "common.h",
        "downsampling.h",
    ],
    deps = [
        "//cyber",
        "//modules/perception/base:point_cloud",
        "//modules/perception/common/geometry:basic",
    ],
)

cc_library(
    name = "point_cloud_processing_test",
    srcs = [
        "common_test.cc",
        "downsampling_test.cc",
    ],
    deps = [
        ":point_cloud_processing",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
