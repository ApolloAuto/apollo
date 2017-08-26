load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mobileye_message_manager",
    srcs = [
    ],
    hdrs = [
        "mobileye_message_manager.h",
    ],
    deps = [
        "//modules/drivers:sensor_message_manager_base",
        "//modules/drivers/mobileye/protocol:drivers_mobileye_protocol",
    ],
)

cc_test(
    name = "mobileye_message_manager_test",
    size = "small",
    srcs = [
        "mobileye_message_manager_test.cc",
    ],
    deps = [
        "mobileye_message_manager",
        "@gtest//:main",
    ],
)

cc_library(
    name = "mobileye_canbus_lib",
    srcs = [
    ],
    hdrs = [
        "mobileye_canbus.h",
    ],
    deps = [
        "//modules/drivers:sensor_canbus_lib",
        "//modules/drivers/mobileye/protocol:drivers_mobileye_protocol",
    ],
)

cpplint()

