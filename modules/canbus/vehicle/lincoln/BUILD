load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lincoln_vehicle_factory",
    srcs = [
        "lincoln_vehicle_factory.cc",
    ],
    hdrs = [
        "lincoln_vehicle_factory.h",
    ],
    deps = [
        ":lincoln_controller",
        ":lincoln_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "lincoln_message_manager",
    srcs = [
        "lincoln_message_manager.cc",
    ],
    hdrs = [
        "lincoln_message_manager.h",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
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
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "//modules/common/kv_db",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "@gtest//:gtest",
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

cc_test(
    name = "lincoln_vehicle_factory_test",
    size = "small",
    srcs = ["lincoln_vehicle_factory_test.cc"],
    deps = [
        ":lincoln_vehicle_factory",
        "@gtest//:main",
    ],
)

cpplint()
