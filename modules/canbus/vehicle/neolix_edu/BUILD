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
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
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
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/neolix_edu/protocol:canbus_neolix_edu_protocol",
    ],
)

cc_test(
    name = "neolix_edu_controller_test",
    size = "small",
    srcs = ["neolix_edu_controller_test.cc"],
    data = ["//modules/canbus:canbus_testdata"],
    deps = [
        ":neolix_edu_controller",
        "@gtest//:main",
    ],
)

cc_test(
    name = "neolix_edu_message_manager_test",
    size = "small",
    srcs = ["neolix_edu_message_manager_test.cc"],
    deps = [
        "//modules/canbus/vehicle/neolix_edu:neolix_edu_message_manager",
        "@gtest//:main",
    ],
)

cc_test(
    name = "neolix_edu_vehicle_factory_test",
    size = "small",
    srcs = ["neolix_edu_vehicle_factory_test.cc"],
    deps = [
        ":neolix_edu_vehicle_factory",
        "@gtest//:main",
    ],
)

cpplint()
