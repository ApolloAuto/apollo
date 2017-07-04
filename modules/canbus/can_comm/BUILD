package(default_visibility = ["//visibility:public"])

cc_library(
    name = "can_receiver",
    srcs = ["can_receiver.cc"],
    hdrs = ["can_receiver.h"],
    deps = [
        "//modules/canbus/can_client",
        "//modules/canbus/vehicle:message_manager_base",
        "//modules/common",
        "//modules/common/proto:error_code_proto",
    ],
)

cc_library(
    name = "can_sender",
    srcs = ["can_sender.cc"],
    hdrs = ["can_sender.h"],
    deps = [
        "//modules/canbus/can_client",
        "//modules/canbus/vehicle:message_manager_base",
        "//modules/common",
        "//modules/common/proto:error_code_proto",
    ],
)

cc_test(
    name = "can_sender_test",
    size = "small",
    srcs = [
        "can_sender_test.cc",
    ],
    deps = [
        "//modules/canbus/can_client/fake:fake_can_client",
        "//modules/canbus/can_comm:can_sender",
        "//modules/canbus/common:canbus_common",
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
        "//modules/canbus/can_client/fake:fake_can_client",
        "//modules/canbus/can_comm:can_receiver",
        "//modules/canbus/common:canbus_common",
        "//modules/common:log",
        "@gtest//:main",
    ],
)
