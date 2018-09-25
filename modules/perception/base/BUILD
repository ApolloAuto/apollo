load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "point_cloud",
    hdrs = [
        "point.h",
        "point_cloud.h",
        "point_cloud_types.h",
    ],
    deps = [
        "@eigen",
    ],
)

cc_test(
    name = "point_cloud_test",
    size = "small",
    srcs = [
        "point_cloud_test.cc",
    ],
    deps = [
        ":point_cloud",
        "@gtest//:main",
    ],
)

cpplint()
