load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

cc_library(
    name = "gem_vehicle_factory",
    srcs = ["gem_vehicle_factory.cc"],
    hdrs = ["gem_vehicle_factory.h"],
    copts = CANBUS_COPTS,
    deps = [
        ":gem_controller",
        ":gem_message_manager",
        "//modules/canbus/proto:canbus_conf_cc_proto",
        "//modules/canbus/proto:vehicle_parameter_cc_proto",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_test(
    name = "gem_vehicle_factory_test",
    size = "small",
    srcs = ["gem_vehicle_factory_test.cc"],
    deps = [
        ":gem_vehicle_factory",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "gem_message_manager",
    srcs = ["gem_message_manager.cc"],
    hdrs = ["gem_message_manager.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/canbus/vehicle/gem/protocol:canbus_gem_protocol",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "gem_message_manager_test",
    size = "small",
    srcs = ["gem_message_manager_test.cc"],
    deps = [
        ":gem_message_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "gem_controller",
    srcs = ["gem_controller.cc"],
    hdrs = ["gem_controller.h"],
    copts = CANBUS_COPTS,
    deps = [
        ":gem_message_manager",
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/gem/protocol:canbus_gem_protocol",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "gem_controller_test",
    size = "small",
    srcs = ["gem_controller_test.cc"],
    data = ["//modules/canbus:canbus_testdata"],
    deps = [
        ":gem_controller",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
