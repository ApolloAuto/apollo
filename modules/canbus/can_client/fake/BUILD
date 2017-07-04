load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "fake_can_client",
    srcs = [
        "fake_can_client.cc",
    ],
    hdrs = [
        "fake_can_client.h",
    ],
    deps = [
        "//modules/canbus/can_client",
    ],
)

cc_test(
    name = "fake_can_client_test",
    size = "small",
    srcs = [
        "fake_can_client_test.cc",
    ],
    deps = [
        "//modules/canbus/can_client/fake:fake_can_client",
        "//modules/canbus/common:canbus_common",
        "//modules/common:log",
        "@gtest//:main",
    ],
)

cpplint()
