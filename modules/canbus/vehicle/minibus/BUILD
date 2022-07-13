load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "minibus_vehicle_factory",
    srcs = [
        "minibus_vehicle_factory.cc",
    ],
    hdrs = [
        "minibus_vehicle_factory.h",
    ],
    deps = [
        ":minibus_controller",
        ":minibus_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "minibus_message_manager",
    srcs = [
        "minibus_message_manager.cc",
    ],
    hdrs = [
        "minibus_message_manager.h",
    ],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle/minibus/protocol:canbus_minibus_protocol",
    ],
)

cc_library(
    name = "minibus_controller",
    srcs = [
        "minibus_controller.cc",
    ],
    hdrs = [
        "minibus_controller.h",
    ],
    deps = [
        ":minibus_message_manager",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/minibus/protocol:canbus_minibus_protocol",
    ],
)

cpplint()
