load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hunter2_vehicle_factory",
    srcs = [
        "hunter2_vehicle_factory.cc",
    ],
    hdrs = [
        "hunter2_vehicle_factory.h",
    ],
    deps = [
        ":hunter2_controller",
        ":hunter2_message_manager",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
    ],
)

cc_library(
    name = "hunter2_message_manager",
    srcs = [
        "hunter2_message_manager.cc",
    ],
    hdrs = [
        "hunter2_message_manager.h",
    ],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle/hunter2/protocol:canbus_hunter2_protocol",
    ],
)

cc_library(
    name = "hunter2_controller",
    srcs = [
        "hunter2_controller.cc",
    ],
    hdrs = [
        "hunter2_controller.h",
    ],
    deps = [
        ":hunter2_message_manager",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:chassis_detail_cc_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus/vehicle/hunter2/protocol:canbus_hunter2_protocol",
    ],
)

cpplint()
