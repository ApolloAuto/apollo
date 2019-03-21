load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_ch_protocol",
    srcs = glob([
        "brake_command_111.cc",
        "brake_status__511.cc",
        "control_command_115.cc",
        "ecu_status_1_515.cc",
        "ecu_status_2_516.cc",
        "ecu_status_3_517.cc",
        "gear_command_114.cc",
        "gear_status_514.cc",
        "steer_command_112.cc",
        "steer_status__512.cc",
        "throttle_command_110.cc",
        "throttle_status__510.cc",
        "turnsignal_command_113.cc",
        "turnsignal_status__513.cc",
    ]),
    hdrs = glob([
        "brake_command_111.h",
        "brake_status__511.h",
        "control_command_115.h",
        "ecu_status_1_515.h",
        "ecu_status_2_516.h",
        "ecu_status_3_517.h",
        "gear_command_114.h",
        "gear_status_514.h",
        "steer_command_112.h",
        "steer_status__512.h",
        "throttle_command_110.h",
        "throttle_status__510.h",
        "turnsignal_command_113.h",
        "turnsignal_status__513.h",
    ]),
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
    ],
)

cc_test(
    name = "brake_command_111_test",
    size = "small",
    srcs = [
        "brake_command_111_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "brake_status__511_test",
    size = "small",
    srcs = [
        "brake_status__511_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "control_command_115_test",
    size = "small",
    srcs = [
        "control_command_115_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "ecu_status_1_515_test",
    size = "small",
    srcs = [
        "ecu_status_1_515_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "ecu_status_2_516_test",
    size = "small",
    srcs = [
        "ecu_status_2_516_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "ecu_status_3_517_test",
    size = "small",
    srcs = [
        "ecu_status_3_517_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "gear_command_114_test",
    size = "small",
    srcs = [
        "gear_command_114_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "gear_status_514_test",
    size = "small",
    srcs = [
        "gear_status_514_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "steer_command_112_test",
    size = "small",
    srcs = [
        "steer_command_112_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "steer_status__512_test",
    size = "small",
    srcs = [
        "steer_status__512_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "throttle_command_110_test",
    size = "small",
    srcs = [
        "throttle_command_110_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "throttle_status__510_test",
    size = "small",
    srcs = [
        "throttle_status__510_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "turnsignal_command_113_test",
    size = "small",
    srcs = [
        "turnsignal_command_113_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@gtest//:main",
    ],
)

cc_test(
    name = "turnsignal_status__513_test",
    size = "small",
    srcs = [
        "turnsignal_status__513_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@gtest//:main",
    ],
)

cpplint()
