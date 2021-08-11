load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools/platform:build_defs.bzl", "if_esd_can")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "esd_can_client",
    srcs = if_esd_can(["esd_can_client.cc"]),
    hdrs = if_esd_can(["esd_can_client.h"]),
    deps = if_esd_can([
        "//modules/common/proto:error_code_cc_proto",
        "//modules/drivers/canbus:sensor_gflags",
        "//modules/drivers/canbus/can_client",
        "//third_party/can_card_library/esd_can",
    ]),
)

cc_test(
    name = "esd_can_client_test",
    size = "small",
    srcs = if_esd_can(["esd_can_client_test.cc"]),
    deps = if_esd_can([
        ":esd_can_client",
        "//cyber",
        "//modules/drivers/canbus/common:canbus_common",
    ]) + [
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
