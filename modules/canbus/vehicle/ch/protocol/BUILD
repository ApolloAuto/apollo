load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "canbus_ch_protocol",
    deps = [
        ":brake_command_111",
        ":brake_status__511",
        ":control_command_115",
        ":ecu_status_1_515",
        ":ecu_status_2_516",
        ":ecu_status_3_517",
        ":gear_command_114",
        ":gear_status_514",
        ":steer_command_112",
        ":steer_status__512",
        ":throttle_command_110",
        ":throttle_status__510",
        ":turnsignal_command_113",
        ":turnsignal_status__513",
    ],
)

cc_library(
    name = "brake_command_111",
    srcs = ["brake_command_111.cc"],
    hdrs = ["brake_command_111.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "brake_command_111_test",
    size = "small",
    srcs = ["brake_command_111_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "brake_status__511",
    srcs = ["brake_status__511.cc"],
    hdrs = ["brake_status__511.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "brake_status__511_test",
    size = "small",
    srcs = ["brake_status__511_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "control_command_115",
    srcs = ["control_command_115.cc"],
    hdrs = ["control_command_115.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "control_command_115_test",
    size = "small",
    srcs = ["control_command_115_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "ecu_status_1_515",
    srcs = ["ecu_status_1_515.cc"],
    hdrs = ["ecu_status_1_515.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "ecu_status_1_515_test",
    size = "small",
    srcs = ["ecu_status_1_515_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "ecu_status_2_516",
    srcs = ["ecu_status_2_516.cc"],
    hdrs = ["ecu_status_2_516.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "ecu_status_2_516_test",
    size = "small",
    srcs = ["ecu_status_2_516_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "ecu_status_3_517",
    srcs = ["ecu_status_3_517.cc"],
    hdrs = ["ecu_status_3_517.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "ecu_status_3_517_test",
    size = "small",
    srcs = ["ecu_status_3_517_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "gear_command_114",
    srcs = ["gear_command_114.cc"],
    hdrs = ["gear_command_114.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "gear_command_114_test",
    size = "small",
    srcs = ["gear_command_114_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "gear_status_514",
    srcs = ["gear_status_514.cc"],
    hdrs = ["gear_status_514.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "gear_status_514_test",
    size = "small",
    srcs = ["gear_status_514_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "steer_command_112",
    srcs = ["steer_command_112.cc"],
    hdrs = ["steer_command_112.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "steer_command_112_test",
    size = "small",
    srcs = ["steer_command_112_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "steer_status__512",
    srcs = ["steer_status__512.cc"],
    hdrs = ["steer_status__512.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "steer_status__512_test",
    size = "small",
    srcs = ["steer_status__512_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "throttle_command_110",
    srcs = ["throttle_command_110.cc"],
    hdrs = ["throttle_command_110.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "throttle_command_110_test",
    size = "small",
    srcs = ["throttle_command_110_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "throttle_status__510",
    srcs = ["throttle_status__510.cc"],
    hdrs = ["throttle_status__510.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "throttle_status__510_test",
    size = "small",
    srcs = ["throttle_status__510_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "turnsignal_command_113",
    srcs = ["turnsignal_command_113.cc"],
    hdrs = ["turnsignal_command_113.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "turnsignal_command_113_test",
    size = "small",
    srcs = ["turnsignal_command_113_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "turnsignal_status__513",
    srcs = ["turnsignal_status__513.cc"],
    hdrs = ["turnsignal_status__513.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "turnsignal_status__513_test",
    size = "small",
    srcs = ["turnsignal_status__513_test.cc"],
    deps = [
        "//modules/canbus/vehicle/ch/protocol:canbus_ch_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
