load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "can_receiver",
    srcs = [],
    hdrs = ["can_receiver.h"],
    deps = [
        "//modules/canbus/can_client",
        "//modules/common",
        "//modules/common/proto:error_code_proto",
        "//modules/drivers:sensor_message_manager_base",
    ],
)

cc_test(
    name = "can_receiver_test",
    size = "small",
    srcs = [
        "can_receiver_test.cc",
    ],
    deps = [
        "can_receiver",
        "//modules/canbus/can_client/fake:fake_can_client",
        "//modules/canbus/common:canbus_common",
        "//modules/common:log",
        "//modules/drivers/mobileye:mobileye_message_manager",
        "//modules/drivers/mobileye/protocol:drivers_mobileye_protocol",
        "@gtest//:main",
    ],
)

cpplint()
