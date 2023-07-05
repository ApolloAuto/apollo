load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

cc_library(
    name = "vehicle_controller_base",
    hdrs = ["vehicle_controller.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus/proto:canbus_conf_cc_proto",
        "//modules/canbus/proto:vehicle_parameter_cc_proto",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/drivers/canbus:sensor_canbus_lib",
    ],
)

cc_library(
    name = "abstract_vehicle_factory",
    srcs = ["abstract_vehicle_factory.cc"],
    hdrs = ["abstract_vehicle_factory.h"],
    copts = CANBUS_COPTS,
    deps = [
        "//cyber",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
    ],
)

cpplint()
