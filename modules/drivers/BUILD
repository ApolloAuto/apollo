load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "sensor_message_manager_base",
    srcs = [
    ],
    hdrs = [
        "sensor_protocol_data.h",
        "sensor_message_manager.h",
    ],
    deps = [
        "//modules/canbus/common:canbus_common",
        "//modules/common/proto:error_code_proto",
        "//modules/common/time",
    ],
)

cc_library(
    name = "mobileye_message_manager",
    srcs = [
    ],
    hdrs = [
        "mobileye_message_manager.h",
    ],
    deps = [
        "sensor_message_manager_base",
        "//modules/drivers/mobileye/protocol:drivers_mobileye_protocol",
    ],
)

cc_test(
    name = "sensor_protocol_data_test",
    size = "small",
    srcs = [
        "sensor_protocol_data_test.cc",
    ],
    deps = [
        "//modules/drivers:sensor_message_manager_base",
	    "@gtest//:main",
    ],
)

cc_test(
    name = "sensor_message_manager_test",
    size = "small",
    srcs = [
        "sensor_message_manager_test.cc",
    ],
    deps = [
        "//modules/drivers:sensor_message_manager_base",
        "//modules/drivers/proto:sensor_proto",
	    "@gtest//:main",
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
    name = "sensor_canbus_lib",
    hdrs = ["sensor_canbus.h"],
    deps = [
        ":sensor_message_manager_base",
        "//modules/canbus:canbus_lib",
        "//modules/drivers/can_comm:can_receiver",
    ],
)

cc_test(
    name = "sensor_canbus_test",
    size = "small",
    srcs = ["sensor_canbus_test.cc"],
    deps = [
        ":sensor_canbus_lib",
        "//modules/drivers/proto:sensor_proto",
        "@gtest//:main",
    ],
)

cpplint()
