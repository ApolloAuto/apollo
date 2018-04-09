load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "vehicle_controller_base",
    srcs = ["vehicle_controller.cc"],
    hdrs = [
        "vehicle_controller.h",
    ],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/proto:error_code_proto",
        "//modules/control/proto:control_proto",
        "//modules/drivers/canbus/can_client",
        "//modules/drivers/canbus/can_comm:can_sender",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
        "//modules/common/configs:vehicle_config_helper",
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
        "//modules/canbus/proto:canbus_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
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
        "//modules/canbus/vehicle/gem:gem_vehicle_factory",
        "//modules/common/util:factory",
    ],
)

cpplint()
