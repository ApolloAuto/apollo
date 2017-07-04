package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lincoln_message_manager",
    srcs = [
        "lincoln_message_manager.cc",
    ],
    hdrs = [
        "lincoln_message_manager.h",
    ],
    deps = [
        "//modules/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle:message_manager_base",
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
    ],
)

cc_library(
    name = "lincoln_controller",
    srcs = [
        "lincoln_controller.cc",
    ],
    hdrs = [
        "lincoln_controller.h",
    ],
    deps = [
        ":lincoln_message_manager",
        "//modules/canbus/can_comm:can_sender",
        "//modules/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
    ],
)

cc_test(
    name = "lincoln_controller_test",
    size = "small",
    srcs = [
        "lincoln_controller_test.cc",
    ],
    data = ["//modules/canbus:canbus_testdata"],
    deps = [
        "//modules/canbus/vehicle/lincoln:lincoln_controller",
        "@gtest//:main",
    ],
)

cc_test(
    name = "lincoln_message_manager_test",
    size = "small",
    srcs = [
        "lincoln_message_manager_test.cc",
    ],
    deps = [
        "//modules/canbus/vehicle/lincoln:lincoln_message_manager",
        "@gtest//:main",
    ],
)
