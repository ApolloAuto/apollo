load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "zhongyun_vehicle_factory",
    srcs = [
        "zhongyun_vehicle_factory.cc",
    ],
    hdrs = [
        "zhongyun_vehicle_factory.h",
    ],
    deps = [
        ":zhongyun_controller",
        ":zhongyun_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "zhongyun_message_manager",
    srcs = [
        "zhongyun_message_manager.cc",
    ],
    hdrs = [
        "zhongyun_message_manager.h",
    ],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle/zhongyun/protocol:canbus_zhongyun_protocol",
    ],
)

cc_library(
    name = "zhongyun_controller",
    srcs = [
        "zhongyun_controller.cc",
    ],
    hdrs = [
        "zhongyun_controller.h",
    ],
    deps = [
        ":zhongyun_message_manager",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/zhongyun/protocol:canbus_zhongyun_protocol",
    ],
)

cc_test(
    name = "zhongyun_message_manager_test",
    size = "small",
    srcs = [
        "zhongyun_message_manager_test.cc",
    ],
    data = ["//modules/canbus:canbus_testdata"],
    deps = [
        ":zhongyun_message_manager",
        "@gtest//:main",
    ],
)

cc_test(
    name = "zhongyun_vehicle_factory_test",
    size = "small",
    srcs = [
        "zhongyun_vehicle_factory_test.cc",
    ],
    data = ["//modules/canbus:canbus_testdata"],
    deps = [
        ":zhongyun_vehicle_factory",
        "@gtest//:main",
    ],
)

cc_test(
    name = "zhongyun_controller_test",
    size = "small",
    srcs = [
        "zhongyun_controller_test.cc",
    ],
    data = ["//modules/canbus:canbus_testdata"],
    deps = [
        ":zhongyun_controller",
        "//modules/common/util",
        "@gtest//:main",
    ],
)

cpplint()
