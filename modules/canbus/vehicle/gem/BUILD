load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gem_vehicle_factory",
    srcs = [
        "gem_vehicle_factory.cc",
    ],
    hdrs = [
        "gem_vehicle_factory.h",
    ],
    deps = [
        ":gem_controller",
        ":gem_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "gem_message_manager",
    srcs = [
        "gem_message_manager.cc",
    ],
    hdrs = [
        "gem_message_manager.h",
    ],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle/gem/protocol:canbus_gem_protocol",
    ],
)

cc_library(
    name = "gem_controller",
    srcs = [
        "gem_controller.cc",
    ],
    hdrs = [
        "gem_controller.h",
    ],
    deps = [
        ":gem_message_manager",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/gem/protocol:canbus_gem_protocol",
    ],
)

cpplint()
