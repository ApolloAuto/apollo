load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "devkit_vehicle_factory",
    srcs = [
        "devkit_vehicle_factory.cc",
    ],
    hdrs = [
        "devkit_vehicle_factory.h",
    ],
    deps = [
        ":devkit_controller",
        ":devkit_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_test(
    name = "devkit_vehicle_factory_test",
    size = "small",
    srcs = ["devkit_vehicle_factory_test.cc"],
    deps = [
        ":devkit_vehicle_factory",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "devkit_message_manager",
    srcs = ["devkit_message_manager.cc"],
    hdrs = ["devkit_message_manager.h"],
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/canbus/vehicle/devkit/protocol:canbus_devkit_protocol",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "devkit_message_manager_test",
    size = "small",
    srcs = ["devkit_message_manager_test.cc"],
    deps = [
        "//modules/canbus/vehicle/devkit:devkit_message_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "devkit_controller",
    srcs = ["devkit_controller.cc"],
    hdrs = ["devkit_controller.h"],
    deps = [
        ":devkit_message_manager",
        "//modules/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_conf_cc_proto",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/canbus/proto:vehicle_parameter_cc_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/devkit/protocol:canbus_devkit_protocol",
        "//modules/common/proto:error_code_cc_proto",
        "//modules/control/proto:control_cmd_cc_proto",
    ],
)

cc_test(
    name = "devkit_controller_test",
    size = "small",
    srcs = ["devkit_controller_test.cc"],
    data = ["//modules/canbus:canbus_testdata"],
    deps = [
        ":devkit_controller",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
