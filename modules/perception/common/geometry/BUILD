load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "basic",
    hdrs = [
        "basic.h",
    ],
    deps = [
        ":common",
        "@eigen",
    ],
)

cc_library(
    name = "common",
    hdrs = [
        "common.h",
    ],
    deps = [
        "//modules/perception/base:box",
        "//modules/perception/base:point_cloud",
    ],
)


cc_library(
    name = "convex_hull_2d",
    hdrs = [
        "convex_hull_2d.h",
    ],
    deps = [
        "@eigen",
    ],
)

cc_test(
    name = "basic_test",
    size = "small",
    srcs = [
        "basic_test.cc",
    ],
    deps = [
        ":basic",
        "@gtest//:main",
    ],
)

cpplint()
