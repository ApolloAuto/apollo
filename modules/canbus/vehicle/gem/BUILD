load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gem_vehicle_factory",
    srcs = ["gem_vehicle_factory.cc"],
    hdrs = ["gem_vehicle_factory.h"],
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        ":gem_controller",
        ":gem_message_manager",
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
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        "//modules/canbus/proto:canbus_proto",
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
    copts = ["-DMODULE_NAME=\\\"canbus\\\""],
    deps = [
        ":gem_message_manager",
        "//modules/canbus/proto:canbus_proto",
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
