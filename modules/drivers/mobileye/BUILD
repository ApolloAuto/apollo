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
        "//modules/drivers/canbus/can_comm:message_manager_base",
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
        ":mobileye_message_manager",
        "//modules/drivers/canbus:sensor_canbus_lib",
        "//modules/drivers/mobileye/protocol:drivers_mobileye_protocol",
    ],
)

cc_test(
    name = "mobileye_canbus_test",
    size = "small",
    srcs = ["mobileye_canbus_test.cc"],
    deps = [
        ":mobileye_canbus_lib",
        ":mobileye_message_manager",
        "//modules/drivers/proto:sensor_proto",
        "@gtest//:main",
    ],
)

cc_binary(
    name = "mobileye",
    srcs = ["main.cc"],
    deps = [
        ":mobileye_canbus_lib",
        "//external:gflags",
        "//modules/canbus/common:canbus_common",
        "//modules/common:log",
        "//modules/common/monitor",
        "@ros//:ros_common",
    ],
)

cpplint()
