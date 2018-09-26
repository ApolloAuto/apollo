load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rt_common",
    srcs = [
        "rt_common.cc",
    ],
    hdrs = [
        "rt_common.h",
    ],
    deps = [
        "//modules/perception/base:common",
        "//modules/perception/proto:rt_proto",
        "//framework:cybertron",
        "@tensorrt",
    ],
)

cc_test(
    name = "rt_common_test",
    size = "small",
    srcs = ["rt_common_test.cc"],
    deps = [
        ":rt_common",
        "@gtest//:main",
    ],
)

cpplint()
