load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ge3_vehicle_factory",
    srcs = ["ge3_vehicle_factory.cc"],
    hdrs = ["ge3_vehicle_factory.h"],
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
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "ge3_message_manager",
    srcs = ["ge3_message_manager.cc"],
    hdrs = ["ge3_message_manager.h"],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle/ge3/protocol:canbus_ge3_protocol_pc_bcm_201",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "ge3_message_manager_test",
    size = "small",
    srcs = ["ge3_message_manager_test.cc"],
    deps = [
        ":ge3_message_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "ge3_controller",
    srcs = ["ge3_controller.cc"],
    hdrs = ["ge3_controller.h"],
    deps = [
        ":ge3_message_manager",
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/ge3/protocol:canbus_ge3_protocol_pc_bcm_201",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "ge3_controller_test",
    size = "small",
    srcs = ["ge3_controller_test.cc"],
    data = ["//modules/canbus:canbus_testdata"],
    deps = [
        ":ge3_controller",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
