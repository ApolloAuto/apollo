load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "neolix_edu_vehicle_factory",
    srcs = [
        "neolix_edu_vehicle_factory.cc",
    ],
    hdrs = [
        "neolix_edu_vehicle_factory.h",
    ],
    deps = [
        ":neolix_edu_controller",
        ":neolix_edu_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "neolix_edu_message_manager",
    srcs = [
        "neolix_edu_message_manager.cc",
    ],
    hdrs = [
        "neolix_edu_message_manager.h",
    ],
    deps = [
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_library(
    name = "neolix_edu_controller",
    srcs = [
        "neolix_edu_controller.cc",
    ],
    hdrs = [
        "neolix_edu_controller.h",
    ],
    deps = [
        ":neolix_edu_message_manager",
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cc_test(
    name = "neolix_edu_controller_test",
    size = "small",
    srcs = ["neolix_edu_controller_test.cc"],
    data = ["//modules/canbus:canbus_testdata"],
    deps = [
        ":neolix_edu_controller",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "neolix_edu_message_manager_test",
    size = "small",
    srcs = ["neolix_edu_message_manager_test.cc"],
    deps = [
        "//modules/canbus/vehicle/neolix_edu:neolix_edu_message_manager",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "neolix_edu_vehicle_factory_test",
    size = "small",
    srcs = ["neolix_edu_vehicle_factory_test.cc"],
    deps = [
        ":neolix_edu_vehicle_factory",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
