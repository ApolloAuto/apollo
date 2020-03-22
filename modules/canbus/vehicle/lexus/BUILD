load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lexus_vehicle_factory",
    srcs = ["lexus_vehicle_factory.cc"],
    hdrs = ["lexus_vehicle_factory.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        ":lexus_controller",
        ":lexus_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "lexus_message_manager",
    srcs = ["lexus_message_manager.cc"],
    hdrs = ["lexus_message_manager.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle/lexus/protocol:canbus_lexus_protocol",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_library(
    name = "lexus_controller",
    srcs = ["lexus_controller.cc"],
    hdrs = ["lexus_controller.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        ":lexus_message_manager",
        "//modules/canbus/proto:canbus_proto",
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
    data = ["//modules/canbus:canbus_testdata"],
    deps = [
        "//modules/canbus/vehicle/lexus:lexus_controller",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
