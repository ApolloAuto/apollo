load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

apollo_cc_library(
    name = "apollo_canbus_vehicle_%(car_type_lower)s",
    srcs = [
        "%(car_type_lower)s_controller.cc",
        "%(car_type_lower)s_message_manager.cc",
        "%(car_type_lower)s_vehicle_factory.cc",
        %(control_cpp_list)s
        %(report_cpp_list)s
    ],
    hdrs = [
        "%(car_type_lower)s_controller.h",
        "%(car_type_lower)s_message_manager.h",
        "%(car_type_lower)s_vehicle_factory.h",
        %(control_header_list)s
        %(report_header_list)s
    ],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus:apollo_canbus",
        "//modules/canbus/proto:canbus_conf_cc_proto",
        "//modules/canbus/proto:vehicle_parameter_cc_proto",
        "//modules/canbus_vehicle/%(car_type_lower)s/proto:%(car_type_lower)s_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/drivers/canbus:apollo_drivers_canbus",
    ],
)

apollo_component(
    name = "lib%(car_type_lower)s_vehicle_factory_lib.so",
    deps = [":apollo_canbus_vehicle_%(car_type_lower)s"],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "testdata/**",
    ]),
)

apollo_package()
cpplint()
