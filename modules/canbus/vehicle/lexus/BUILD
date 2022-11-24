load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

cc_library(
    name = "lexus_vehicle_factory",
    srcs = ["lexus_vehicle_factory.cc"],
    hdrs = ["lexus_vehicle_factory.h"],
    copts = CANBUS_COPTS,
    alwayslink = True,
    deps = [
        ":lexus_controller",
        ":lexus_message_manager",
        "//modules/canbus/common:canbus_common",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:can_sender",
    ],
)

cc_binary(
    name = "liblexus_vehicle_factory_lib.so",
    linkshared = True,
    linkstatic = True,
    deps = [":lexus_vehicle_factory"],
)

cc_library(
    name = "lexus_message_manager",
    srcs = ["lexus_message_manager.cc"],
    hdrs = ["lexus_message_manager.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/vehicle/lexus/proto:lexus_cc_proto",
        "//modules/canbus/vehicle/lexus/protocol:canbus_lexus_protocol",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_library(
    name = "lexus_controller",
    srcs = ["lexus_controller.cc"],
    hdrs = ["lexus_controller.h"],
    copts = CANBUS_COPTS,
    deps = [
        ":lexus_message_manager",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/canbus/vehicle/lexus/proto:lexus_cc_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/lexus/protocol:canbus_lexus_protocol",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "lexus_controller_test",
    size = "small",
    srcs = ["lexus_controller_test.cc"],
    data = ["//modules/canbus:test_data"],
    deps = [
        "//modules/canbus/vehicle/lexus:lexus_controller",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "lexus_vehicle_factory_test",
    size = "small",
    srcs = ["lexus_vehicle_factory_test.cc"],
    data = ["//modules/canbus:test_data"],
    linkstatic = True,
    deps = [
        ":lexus_vehicle_factory",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
