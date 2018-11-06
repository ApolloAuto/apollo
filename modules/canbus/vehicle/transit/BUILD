load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "transit_vehicle_factory",
    srcs = [
        "transit_vehicle_factory.cc",
    ],
    hdrs = [
        "transit_vehicle_factory.h",
    ],
    deps = [
        ":transit_controller",
        ":transit_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "transit_message_manager",
    srcs = [
        "transit_message_manager.cc",
    ],
    hdrs = [
        "transit_message_manager.h",
    ],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle/transit/protocol:canbus_transit_protocol",
    ],
)

cc_library(
    name = "transit_controller",
    srcs = [
        "transit_controller.cc",
    ],
    hdrs = [
        "transit_controller.h",
    ],
    deps = [
        ":transit_message_manager",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/kv_db",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/transit/protocol:canbus_transit_protocol",
    ],
)

cpplint()
