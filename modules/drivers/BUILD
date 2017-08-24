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

cpplint()
