load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_common",
    srcs = [
        "byte.cc",
        "canbus_gflags.cc",
    ],
    hdrs = [
        "byte.h",
        "canbus_consts.h",
        "canbus_gflags.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common:log",
    ],
)

cc_test(
    name = "byte_test",
    size = "small",
    srcs = [
        "byte_test.cc",
    ],
    deps = [
        "//modules/canbus/common:canbus_common",
        "@gtest//:main",
    ],
)

cpplint()
