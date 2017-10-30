load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "esd_can_client",
    srcs = [
        "esd_can_client.cc",
    ],
    hdrs = [
        "esd_can_client.h",
    ],
    deps = [
        "//modules/drivers/canbus/can_client",
        "//third_party/can_card_library/esd_can",
    ],
)

cc_test(
    name = "esd_can_client_test",
    size = "small",
    srcs = [
        "esd_can_client_test.cc",
    ],
    deps = [
        "//modules/common:log",
        "//modules/drivers/canbus/can_client/esd:esd_can_client",
        "//modules/drivers/canbus/common:canbus_common",
        "@gtest//:main",
    ],
)

cpplint()
