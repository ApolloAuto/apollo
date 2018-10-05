load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lexus_vehicle_factory",
    srcs = [
        "lexus_vehicle_factory.cc",
    ],
    hdrs = [
        "lexus_vehicle_factory.h",
    ],
    deps = [
        ":lexus_controller",
        ":lexus_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "lexus_message_manager",
    srcs = [
        "lexus_message_manager.cc",
    ],
    hdrs = [
        "lexus_message_manager.h",
    ],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle/lexus/protocol:canbus_lexus_protocol",
    ],
)

cc_library(
    name = "lexus_controller",
    srcs = [
        "lexus_controller.cc",
    ],
    hdrs = [
        "lexus_controller.h",
    ],
    deps = [
        ":lexus_message_manager",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/lexus/protocol:canbus_lexus_protocol",
    ],
)

cpplint()
