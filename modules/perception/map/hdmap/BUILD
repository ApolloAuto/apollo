load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hdmap_input",
    srcs = [
        "hdmap_input.cc",
    ],
    hdrs = [
        "hdmap_input.h",
    ],
    deps = [
        "//modules/perception/base:base_type",
        "//modules/perception/base:point_cloud",
        "//modules/perception/lib/singleton",
    ],
)

cc_test(
    name = "hdmap_input_test",
    size = "small",
    srcs = [
        "hdmap_input_test.cc",
    ],
    deps = [
        "@gtest//:main",
        ":hdmap_input",
    ],
)

cpplint()
