load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "%(car_type_lower)s_vehicle_factory",
    srcs = [
        "%(car_type_lower)s_vehicle_factory.cc",
    ],
    hdrs = [
        "%(car_type_lower)s_vehicle_factory.h",
    ],
    deps = [
        ":%(car_type_lower)s_controller",
        ":%(car_type_lower)s_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "%(car_type_lower)s_message_manager",
    srcs = [
        "%(car_type_lower)s_message_manager.cc",
    ],
    hdrs = [
        "%(car_type_lower)s_message_manager.h",
    ],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle/%(car_type_lower)s/protocol:canbus_%(car_type_lower)s_protocol",
    ],
)

cc_library(
    name = "%(car_type_lower)s_controller",
    srcs = [
        "%(car_type_lower)s_controller.cc",
    ],
    hdrs = [
        "%(car_type_lower)s_controller.h",
    ],
    deps = [
        ":%(car_type_lower)s_message_manager",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/%(car_type_lower)s/protocol:canbus_%(car_type_lower)s_protocol",
    ],
)

cpplint()
