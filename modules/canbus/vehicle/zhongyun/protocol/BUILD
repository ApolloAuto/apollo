load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

cc_library(
    name = "canbus_zhongyun_protocol",
    deps = [
        "brake_control_a4",
        "enable_state_feedback_c3",
        "error_state_e1",
        "gear_control_a1",
        "parking_control_a5",
        "steering_control_a2",
        "torque_control_a3",
        "vehicle_state_feedback_2_c4",
        "vehicle_state_feedback_c1",
    ],
)

cc_library(
    name = "brake_control_a4",
    srcs = ["brake_control_a4.cc"],
    hdrs = ["brake_control_a4.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "brake_control_a4_test",
    size = "small",
    srcs = ["brake_control_a4_test.cc"],
    deps = [
        ":canbus_zhongyun_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "enable_state_feedback_c3",
    srcs = ["enable_state_feedback_c3.cc"],
    hdrs = ["enable_state_feedback_c3.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "enable_state_feedback_c3_test",
    size = "small",
    srcs = ["enable_state_feedback_c3_test.cc"],
    deps = [
        ":canbus_zhongyun_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "error_state_e1",
    srcs = ["error_state_e1.cc"],
    hdrs = ["error_state_e1.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "error_state_e1_test",
    size = "small",
    srcs = ["error_state_e1_test.cc"],
    deps = [
        ":canbus_zhongyun_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "gear_control_a1",
    srcs = ["gear_control_a1.cc"],
    hdrs = ["gear_control_a1.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "gear_control_a1_test",
    size = "small",
    srcs = ["gear_control_a1_test.cc"],
    deps = [
        ":canbus_zhongyun_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "parking_control_a5",
    srcs = ["parking_control_a5.cc"],
    hdrs = ["parking_control_a5.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "parking_control_a5_test",
    size = "small",
    srcs = ["parking_control_a5_test.cc"],
    deps = [
        ":canbus_zhongyun_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "steering_control_a2",
    srcs = ["steering_control_a2.cc"],
    hdrs = ["steering_control_a2.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "steering_control_a2_test",
    size = "small",
    srcs = ["steering_control_a2_test.cc"],
    deps = [
        ":canbus_zhongyun_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "torque_control_a3",
    srcs = ["torque_control_a3.cc"],
    hdrs = ["torque_control_a3.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "torque_control_a3_test",
    size = "small",
    srcs = ["torque_control_a3_test.cc"],
    deps = [
        ":canbus_zhongyun_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "vehicle_state_feedback_2_c4",
    srcs = ["vehicle_state_feedback_2_c4.cc"],
    hdrs = ["vehicle_state_feedback_2_c4.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "vehicle_state_feedback_2_c4_test",
    size = "small",
    srcs = ["vehicle_state_feedback_2_c4_test.cc"],
    deps = [
        ":canbus_zhongyun_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "vehicle_state_feedback_c1",
    srcs = ["vehicle_state_feedback_c1.cc"],
    hdrs = ["vehicle_state_feedback_c1.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "vehicle_state_feedback_c1_test",
    size = "small",
    srcs = ["vehicle_state_feedback_c1_test.cc"],
    deps = [
        ":canbus_zhongyun_protocol",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
