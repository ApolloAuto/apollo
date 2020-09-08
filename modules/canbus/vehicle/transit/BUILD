load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

cc_library(
    name = "transit_vehicle_factory",
    srcs = ["transit_vehicle_factory.cc"],
    hdrs = ["transit_vehicle_factory.h"],
    copts = CANBUS_COPTS,
    deps = [
        ":transit_controller",
        ":transit_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "transit_message_manager",
    srcs = ["transit_message_manager.cc"],
    hdrs = ["transit_message_manager.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/canbus/vehicle/transit/protocol:canbus_transit_protocol",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_library(
    name = "transit_controller",
    srcs = ["transit_controller.cc"],
    hdrs = ["transit_controller.h"],
    copts = CANBUS_COPTS,
    deps = [
        ":transit_message_manager",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/common/kv_db",
        "//modules/common/proto:vehicle_signal_cc_proto",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "transit_vehicle_factory_test",
    size = "small",
    srcs = ["transit_vehicle_factory_test.cc"],
    deps = [
        ":transit_vehicle_factory",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "transit_controller_test",
    size = "small",
    srcs = ["transit_controller_test.cc"],
    data = ["//modules/canbus:canbus_testdata"],
    deps = [
        ":transit_controller",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
