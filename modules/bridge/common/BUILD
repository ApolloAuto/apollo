load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "bridge_buffer",
    srcs = [
        "bridge_buffer.cc",
    ],
    hdrs = [
        "bridge_buffer.h",
    ],
)

cc_library(
    name = "bridge_gflags",
    srcs = [
        "bridge_gflags.cc",
    ],
    hdrs = [
        "bridge_gflags.h",
    ],
)

cc_test(
    name = "bridge_buffer_test",
    size = "small",
    srcs = [
        "bridge_buffer_test.cc",
    ],
    deps = [
        "bridge_buffer",
        "@gtest//:main",
    ],
)

cpplint()
