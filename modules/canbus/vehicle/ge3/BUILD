load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ge3_vehicle_factory",
    srcs = [
        "ge3_vehicle_factory.cc",
    ],
    hdrs = [
        "ge3_vehicle_factory.h",
    ],
    deps = [
        ":ge3_controller",
        ":ge3_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_test(
    name = "ge3_vehicle_factory_test",
    size = "small",
    srcs = ["ge3_vehicle_factory_test.cc"],
    deps = [
        ":ge3_vehicle_factory",
        "@gtest//:main",
    ],
)

cc_library(
    name = "ge3_message_manager",
    srcs = [
        "ge3_message_manager.cc",
    ],
    hdrs = [
        "ge3_message_manager.h",
    ],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle/ge3/protocol:canbus_ge3_protocol_pc_bcm_201",
    ],
)

cc_test(
    name = "ge3_message_manager_test",
    size = "small",
    srcs = [
        "ge3_message_manager_test.cc",
    ],
    deps = [
        ":ge3_message_manager",
        "@gtest//:main",
    ],
)

cc_library(
    name = "ge3_controller",
    srcs = [
        "ge3_controller.cc",
    ],
    hdrs = [
        "ge3_controller.h",
    ],
    deps = [
        ":ge3_message_manager",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/ge3/protocol:canbus_ge3_protocol_pc_bcm_201",
    ],
)

cc_test(
    name = "ge3_controller_test",
    size = "small",
    data = ["//modules/canbus:canbus_testdata"],
    srcs = [
        "ge3_controller_test.cc",
    ],
    deps = [
        ":ge3_controller",
        "//modules/common/util:util",
        "@gtest//:main",
    ],
)

cpplint()
