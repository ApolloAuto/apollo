load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

cc_library(
    name = "canbus_devkit_protocol",
    deps = [
        ":bms_report_512",
        ":brake_command_101",
        ":brake_report_501",
        ":gear_command_103",
        ":gear_report_503",
        ":park_command_104",
        ":park_report_504",
        ":steering_command_102",
        ":steering_report_502",
        ":throttle_command_100",
        ":throttle_report_500",
        ":ultr_sensor_1_507",
        ":ultr_sensor_2_508",
        ":ultr_sensor_3_509",
        ":ultr_sensor_4_510",
        ":ultr_sensor_5_511",
        ":vcu_report_505",
        ":wheelspeed_report_506",
    ],
)

cc_library(
    name = "bms_report_512",
    srcs = ["bms_report_512.cc"],
    hdrs = ["bms_report_512.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "bms_report_512_test",
    size = "small",
    srcs = ["bms_report_512_test.cc"],
    deps = [
        "//modules/canbus/vehicle/devkit/protocol:canbus_devkit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "brake_command_101",
    srcs = ["brake_command_101.cc"],
    hdrs = ["brake_command_101.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "brake_command_101_test",
    size = "small",
    srcs = ["brake_command_101_test.cc"],
    deps = [
        "//modules/canbus/vehicle/devkit/protocol:canbus_devkit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "brake_report_501",
    srcs = ["brake_report_501.cc"],
    hdrs = ["brake_report_501.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "brake_report_501_test",
    size = "small",
    srcs = ["brake_report_501_test.cc"],
    deps = [
        "//modules/canbus/vehicle/devkit/protocol:canbus_devkit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "gear_command_103",
    srcs = ["gear_command_103.cc"],
    hdrs = ["gear_command_103.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "gear_command_103_test",
    size = "small",
    srcs = ["gear_command_103_test.cc"],
    deps = [
        "//modules/canbus/vehicle/devkit/protocol:canbus_devkit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "gear_report_503",
    srcs = ["gear_report_503.cc"],
    hdrs = ["gear_report_503.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "gear_report_503_test",
    size = "small",
    srcs = ["gear_report_503_test.cc"],
    deps = [
        "//modules/canbus/vehicle/devkit/protocol:canbus_devkit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "park_command_104",
    srcs = ["park_command_104.cc"],
    hdrs = ["park_command_104.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "park_command_104_test",
    size = "small",
    srcs = ["park_command_104_test.cc"],
    deps = [
        "//modules/canbus/vehicle/devkit/protocol:canbus_devkit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "park_report_504",
    srcs = ["park_report_504.cc"],
    hdrs = ["park_report_504.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "park_report_504_test",
    size = "small",
    srcs = ["park_report_504_test.cc"],
    deps = [
        "//modules/canbus/vehicle/devkit/protocol:canbus_devkit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "steering_command_102",
    srcs = ["steering_command_102.cc"],
    hdrs = ["steering_command_102.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "steering_command_102_test",
    size = "small",
    srcs = ["steering_command_102_test.cc"],
    deps = [
        "//modules/canbus/vehicle/devkit/protocol:canbus_devkit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "steering_report_502",
    srcs = ["steering_report_502.cc"],
    hdrs = ["steering_report_502.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "steering_report_502_test",
    size = "small",
    srcs = ["steering_report_502_test.cc"],
    deps = [
        "//modules/canbus/vehicle/devkit/protocol:canbus_devkit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "throttle_command_100",
    srcs = ["throttle_command_100.cc"],
    hdrs = ["throttle_command_100.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "throttle_command_100_test",
    size = "small",
    srcs = ["throttle_command_100_test.cc"],
    deps = [
        "//modules/canbus/vehicle/devkit/protocol:canbus_devkit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "throttle_report_500",
    srcs = ["throttle_report_500.cc"],
    hdrs = ["throttle_report_500.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "throttle_report_500_test",
    size = "small",
    srcs = ["throttle_report_500_test.cc"],
    deps = [
        "//modules/canbus/vehicle/devkit/protocol:canbus_devkit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "ultr_sensor_1_507",
    srcs = ["ultr_sensor_1_507.cc"],
    hdrs = ["ultr_sensor_1_507.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "ultr_sensor_1_507_test",
    size = "small",
    srcs = ["ultr_sensor_1_507_test.cc"],
    deps = [
        "//modules/canbus/vehicle/devkit/protocol:canbus_devkit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "ultr_sensor_2_508",
    srcs = ["ultr_sensor_2_508.cc"],
    hdrs = ["ultr_sensor_2_508.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "ultr_sensor_2_508_test",
    size = "small",
    srcs = ["ultr_sensor_2_508_test.cc"],
    deps = [
        "//modules/canbus/vehicle/devkit/protocol:canbus_devkit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "ultr_sensor_3_509",
    srcs = ["ultr_sensor_3_509.cc"],
    hdrs = ["ultr_sensor_3_509.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "ultr_sensor_3_509_test",
    size = "small",
    srcs = ["ultr_sensor_3_509_test.cc"],
    deps = [
        "//modules/canbus/vehicle/devkit/protocol:canbus_devkit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "ultr_sensor_4_510",
    srcs = ["ultr_sensor_4_510.cc"],
    hdrs = ["ultr_sensor_4_510.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "ultr_sensor_4_510_test",
    size = "small",
    srcs = ["ultr_sensor_4_510_test.cc"],
    deps = [
        "//modules/canbus/vehicle/devkit/protocol:canbus_devkit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "ultr_sensor_5_511",
    srcs = ["ultr_sensor_5_511.cc"],
    hdrs = ["ultr_sensor_5_511.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "ultr_sensor_5_511_test",
    size = "small",
    srcs = ["ultr_sensor_5_511_test.cc"],
    deps = [
        "//modules/canbus/vehicle/devkit/protocol:canbus_devkit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "vcu_report_505",
    srcs = ["vcu_report_505.cc"],
    hdrs = ["vcu_report_505.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "vcu_report_505_test",
    size = "small",
    srcs = ["vcu_report_505_test.cc"],
    deps = [
        "//modules/canbus/vehicle/devkit/protocol:canbus_devkit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "wheelspeed_report_506",
    srcs = ["wheelspeed_report_506.cc"],
    hdrs = ["wheelspeed_report_506.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "wheelspeed_report_506_test",
    size = "small",
    srcs = ["wheelspeed_report_506_test.cc"],
    deps = [
        "//modules/canbus/vehicle/devkit/protocol:canbus_devkit_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
