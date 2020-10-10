load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

cc_library(
    name = "lincoln_vehicle_factory",
    srcs = ["lincoln_vehicle_factory.cc"],
    hdrs = ["lincoln_vehicle_factory.h"],
    copts = CANBUS_COPTS,
    deps = [
        ":lincoln_controller",
        ":lincoln_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "lincoln_message_manager",
    srcs = ["lincoln_message_manager.cc"],
    hdrs = ["lincoln_message_manager.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_library(
    name = "lincoln_controller",
    srcs = ["lincoln_controller.cc"],
    hdrs = ["lincoln_controller.h"],
    copts = CANBUS_COPTS,
    deps = [
        ":lincoln_message_manager",
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/lincoln/protocol:canbus_lincoln_protocol",
        "//modules/common/kv_db",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "lincoln_controller_test",
    size = "small",
    srcs = ["lincoln_controller_test.cc"],
    data = ["//modules/canbus:canbus_testdata"],
    deps = [
        "//modules/canbus/vehicle/lincoln:lincoln_controller",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "lincoln_message_manager_test",
    size = "small",
    srcs = ["lincoln_message_manager_test.cc"],
    deps = [
        "//modules/canbus/vehicle/lincoln:lincoln_message_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "lincoln_vehicle_factory_test",
    size = "small",
    srcs = ["lincoln_vehicle_factory_test.cc"],
    deps = [
        ":lincoln_vehicle_factory",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
