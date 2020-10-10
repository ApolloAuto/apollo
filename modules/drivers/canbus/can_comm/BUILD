load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "can_receiver",
    hdrs = ["can_receiver.h"],
    deps = [
        "//modules/common/proto:error_code_cc_proto",
        "//modules/drivers/canbus/can_client",
        "//modules/drivers/canbus/can_comm:message_manager_base",
    ],
)

cc_library(
    name = "can_sender",
    hdrs = ["can_sender.h"],
    deps = [
        "//modules/common/proto:error_code_cc_proto",
        "//modules/drivers/canbus/can_client",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "message_manager_base",
    hdrs = [
        "message_manager.h",
        "protocol_data.h",
    ],
    deps = [
        "//cyber/common:log",
        "//cyber/time",
        "//modules/common/proto:error_code_cc_proto",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "can_sender_test",
    size = "small",
    srcs = ["can_sender_test.cc"],
    deps = [
        "//cyber",
        "//modules/canbus/proto:ch_cc_proto",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/canbus/proto:ge3_cc_proto",
        "//modules/canbus/proto:lexus_cc_proto",
        "//modules/canbus/proto:transit_cc_proto",
        "//modules/canbus/proto:wey_cc_proto",
        "//modules/canbus/proto:zhongyun_cc_proto",
        "//modules/canbus/proto:neolix_edu_cc_proto",
        "//modules/drivers/canbus/can_client/fake:fake_can_client",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "can_receiver_test",
    size = "small",
    srcs = ["can_receiver_test.cc"],
    deps = [
        "//cyber",
        "//modules/canbus/proto:ch_cc_proto",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/canbus/proto:ge3_cc_proto",
        "//modules/canbus/proto:lexus_cc_proto",
        "//modules/canbus/proto:transit_cc_proto",
        "//modules/canbus/proto:wey_cc_proto",
        "//modules/canbus/proto:zhongyun_cc_proto",
        "//modules/canbus/proto:neolix_edu_cc_proto",
        "//modules/common/configs/proto:vehicle_config_cc_proto",
        "//modules/common/proto:drive_state_cc_proto",
        "//modules/common/proto:header_cc_proto",
        "//modules/common/proto:vehicle_signal_cc_proto",
        "//modules/drivers/canbus/can_client/fake:fake_can_client",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/common:canbus_common",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "protocol_data_test",
    size = "small",
    srcs = ["protocol_data_test.cc"],
    deps = [
        "//modules/canbus/proto:ch_cc_proto",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/canbus/proto:ge3_cc_proto",
        "//modules/canbus/proto:lexus_cc_proto",
        "//modules/canbus/proto:transit_cc_proto",
        "//modules/canbus/proto:wey_cc_proto",
        "//modules/canbus/proto:zhongyun_cc_proto",
        "//modules/canbus/proto:neolix_edu_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "message_manager_test",
    size = "small",
    srcs = ["message_manager_test.cc"],
    deps = [
        "//cyber",
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
