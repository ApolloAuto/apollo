load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
# load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

cc_library(
    name = "%(car_type_lower)s_vehicle_factory",
    srcs = [
        "%(car_type_lower)s_vehicle_factory.cc",
    ],
    hdrs = [
        "%(car_type_lower)s_vehicle_factory.h",
    ],
    copts = CANBUS_COPTS,
    alwayslink = True,
    deps = [
        ":%(car_type_lower)s_controller",
        ":%(car_type_lower)s_message_manager",
        "//modules/canbus/common:canbus_gflags",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/canbus/vehicle:abstract_vehicle_factory",
        "//modules/drivers/canbus:sensor_canbus_lib",
    ],
)

cc_binary(
    name = "lib%(car_type_lower)s_vehicle_factory_lib.so",
    linkshared = True,
    linkstatic = True,
    deps = [":%(car_type_lower)s_vehicle_factory"],
)

cc_library(
    name = "%(car_type_lower)s_message_manager",
    srcs = [
        "%(car_type_lower)s_message_manager.cc",
    ],
    hdrs = [
        "%(car_type_lower)s_message_manager.h",
    ],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus_vehicle/%(car_type_lower)s/proto:%(car_type_lower)s_cc_proto",
        "//modules/canbus_vehicle/%(car_type_lower)s/protocol:canbus_%(car_type_lower)s_protocol",
        "//modules/drivers/canbus/can_comm:message_manager_base",
        "//modules/drivers/canbus/common:canbus_common",
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
    copts = CANBUS_COPTS,
    deps = [
        ":%(car_type_lower)s_message_manager",
        "//modules/canbus/proto:canbus_conf_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/canbus/proto:vehicle_parameter_cc_proto",
        "//modules/canbus/vehicle:vehicle_controller_base",
        "//modules/canbus_vehicle/%(car_type_lower)s/protocol:canbus_%(car_type_lower)s_protocol",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "testdata/**",
    ]),
)

apollo_package()
cpplint()
