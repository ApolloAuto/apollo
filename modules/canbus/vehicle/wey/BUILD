load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

cc_library(
    name = "wey_vehicle_factory",
    srcs = ["wey_vehicle_factory.cc"],
    hdrs = ["wey_vehicle_factory.h"],
    copts = CANBUS_COPTS,
    alwayslink = True,
    deps = [
        ":wey_controller",
        ":wey_message_manager",
        "//modules/canbus/common:canbus_gflags",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
        "//modules/drivers/canbus/can_client:can_client_factory",
        "//modules/drivers/canbus/can_comm:can_receiver",
        "//modules/drivers/canbus/can_comm:can_sender",
    ],
)

cc_binary(
    name = "libwey_vehicle_factory_lib.so",
    linkshared = True,
    linkstatic = True,
    deps = [":wey_vehicle_factory"],
)

cc_test(
    name = "wey_vehicle_factory_test",
    size = "small",
    srcs = ["wey_vehicle_factory_test.cc"],
    data = ["//modules/canbus:test_data"],
    linkstatic = True,
    deps = [
        ":wey_vehicle_factory",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "wey_message_manager",
    srcs = ["wey_message_manager.cc"],
    hdrs = ["wey_message_manager.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/vehicle/wey/proto:wey_cc_proto",
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
    copts = CANBUS_COPTS,
    deps = [
        ":wey_message_manager",
        "//modules/canbus/proto:canbus_conf_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/canbus/proto:vehicle_parameter_cc_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/wey/protocol:canbus_wey_protocol",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
    ],
)

cc_test(
    name = "wey_controller_test",
    size = "small",
    srcs = ["wey_controller_test.cc"],
    data = ["//modules/canbus:test_data"],
    deps = [
        ":wey_controller",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
