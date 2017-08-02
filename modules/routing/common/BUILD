load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    deps = [
        ":routing_gflags",
        ":routing_util",
        ":routing_macros",
    ],
)

cc_library(
    name = "routing_gflags",
    srcs = [
        "routing_gflags.cc",
    ],
    hdrs = [
        "routing_gflags.h",
    ],
    deps = [
        "//external:gflags",
    ],
)

cc_library(
    name = "routing_util",
    srcs = [
        "utils.cc",
    ],
    hdrs = [
        "utils.h",
    ],
    deps = [
        "//modules/common:common",
    ],
)

cc_library(
    name = "routing_macros",
    hdrs = [
        "routing_macros.h",
    ],
)

cc_test(
    name = "utils_test",
    size = "small",
    srcs = [
        "utils_test.cc",
    ],
    deps = [
        ":common",
        "//modules/routing/proto:routing_proto",
        "@gtest//:main",
    ],
)


cpplint()
