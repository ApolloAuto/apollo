load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "point_cloud_processing",
    hdrs = [
        "common.h",
        "downsampling.h",
    ],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/perception/common/base:point_cloud",
        "//modules/perception/common/algorithm/geometry:basic",
    ],
)

cc_library(
    name = "point_cloud_processing_test",
    srcs = [
        "common_test.cc",
        "downsampling_test.cc",
    ],
    alwayslink = True,
    deps = [
        ":point_cloud_processing",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()
cpplint()
