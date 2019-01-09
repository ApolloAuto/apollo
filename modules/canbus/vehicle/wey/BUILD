load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "wey_vehicle_factory",
    srcs = [
        "wey_vehicle_factory.cc",
    ],
    hdrs = [
        "wey_vehicle_factory.h",
    ],
    deps = [
        ":wey_controller",
        ":wey_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_test(
    name = "wey_vehicle_factory_test",
    size = "small",
    srcs = ["wey_vehicle_factory_test.cc"],
    deps = [
        ":wey_vehicle_factory",
        "@gtest//:main",
    ],
)

cc_library(
    name = "wey_message_manager",
    srcs = [
        "wey_message_manager.cc",
    ],
    hdrs = [
        "wey_message_manager.h",
    ],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle/wey/protocol:canbus_wey_protocol",
    ],
)

cc_test(
    name = "wey_message_manager_test",
    size = "small",
    srcs = [
        "wey_message_manager_test.cc",
    ],
    deps = [
        ":wey_message_manager",
        "@gtest//:main",
    ],
)

cc_library(
    name = "wey_controller",
    srcs = [
        "wey_controller.cc",
    ],
    hdrs = [
        "wey_controller.h",
    ],
    deps = [
        ":wey_message_manager",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/wey/protocol:canbus_wey_protocol",
    ],
)

cc_test(
    name = "wey_controller_test",
    size = "small",
    data = ["//modules/canbus:canbus_testdata"],
    srcs = [
        "wey_controller_test.cc",
    ],
    deps = [
        ":wey_controller",
        "//modules/common/util:util",
        "@gtest//:main",
    ],
)

cpplint()
