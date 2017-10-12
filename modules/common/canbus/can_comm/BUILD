load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "can_receiver",
    srcs = ["can_receiver.cc"],
    hdrs = ["can_receiver.h"],
    deps = [
        "//modules/common/canbus/can_client",
        "//modules/common/canbus/can_comm:message_manager_base",
        "//modules/common",
        "//modules/common/proto:error_code_proto",
    ],
)

cc_library(
    name = "can_sender",
    srcs = ["can_sender.cc"],
    hdrs = ["can_sender.h"],
    deps = [
        "//modules/common/canbus/can_client",
        "//modules/common/canbus/can_comm:message_manager_base",
        "//modules/common",
        "//modules/common/proto:error_code_proto",
        "@gtest//:gtest",
    ],
)

cc_library(
    name = "message_manager_base",
    srcs = [
        "message_manager.cc",
        "protocol_data.cc",
    ],
    hdrs = [
        "message_manager.h",
        "protocol_data.h",
    ],
    deps = [
        "//modules/common/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/proto:error_code_proto",
        "//modules/common/time",
        "//modules/control/proto:control_proto",
    ],
)

cc_test(
    name = "can_sender_test",
    size = "small",
    srcs = [
        "can_sender_test.cc",
    ],
    deps = [
        "//modules/common/canbus/can_client/fake:fake_can_client",
        "//modules/common/canbus/can_comm:can_sender",
        "//modules/common/canbus/common:canbus_common",
        "//modules/common:log",
        "@gtest//:main",
    ],
)

cc_test(
    name = "can_receiver_test",
    size = "small",
    srcs = [
        "can_receiver_test.cc",
    ],
    deps = [
        "//modules/common/canbus/can_client/fake:fake_can_client",
        "//modules/common/canbus/can_comm:can_receiver",
        "//modules/common/canbus/common:canbus_common",
        "//modules/common:log",
        "@gtest//:main",
    ],
)

cc_test(
    name = "protocol_data_test",
    size = "small",
    srcs = [
        "protocol_data_test.cc",
    ],
    deps = [
        "//modules/common/canbus/can_comm:message_manager_base",
        "@gtest//:main",
    ],
)

cc_test(
    name = "message_manager_test",
    size = "small",
    srcs = ["message_manager_test.cc"],
    deps = [
        "//modules/common/canbus/can_comm:message_manager_base",
        "@gtest//:main",
    ],
)

cpplint()
