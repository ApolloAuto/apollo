load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "vehicle_controller_base",
    srcs = ["vehicle_controller.cc"],
    hdrs = [
        "vehicle_controller.h",
    ],
    deps = [
        "//modules/drivers/canbus/can_client",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/common/proto:error_code_proto",
        "//modules/control/proto:control_proto",
    ],
)

cc_library(
    name = "abstract_vehicle_factory",
    srcs = [
        "abstract_vehicle_factory.cc",
    ],
    hdrs = [
        "abstract_vehicle_factory.h",
    ],
    deps = [
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/canbus/proto:canbus_proto",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/canbus/vehicle:vehicle_controller_base",
    ],
)

cc_library(
    name = "vehicle_factory",
    srcs = [
        "vehicle_factory.cc",
    ],
    hdrs = [
        "vehicle_factory.h",
    ],
    deps = [
        "//modules/canbus/vehicle/lincoln:lincoln_vehicle_factory",
        "//modules/common/util:factory",
    ],
)

cpplint()
