load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_common",
    srcs = [
        "byte.cc",
    ],
    hdrs = [
        "byte.h",
        "canbus_consts.h",
    ],
    deps = [
    ],
)

cc_test(
    name = "byte_test",
    size = "small",
    srcs = [
        "byte_test.cc",
    ],
    deps = [
        "//modules/common/canbus/common:canbus_common",
        "@gtest//:main",
    ],
)

cpplint()
