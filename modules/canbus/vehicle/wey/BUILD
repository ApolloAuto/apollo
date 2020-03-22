load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "wey_vehicle_factory",
    srcs = ["wey_vehicle_factory.cc"],
    hdrs = ["wey_vehicle_factory.h"],
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
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "wey_message_manager",
    srcs = ["wey_message_manager.cc"],
    hdrs = ["wey_message_manager.h"],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle/wey/protocol:canbus_wey_protocol",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "wey_message_manager_test",
    size = "small",
    srcs = ["wey_message_manager_test.cc"],
    deps = [
        ":wey_message_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "wey_controller",
    srcs = ["wey_controller.cc"],
    hdrs = ["wey_controller.h"],
    deps = [
        ":wey_message_manager",
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/wey/protocol:canbus_wey_protocol",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "wey_controller_test",
    size = "small",
    srcs = ["wey_controller_test.cc"],
    data = ["//modules/canbus:canbus_testdata"],
    deps = [
        ":wey_controller",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
