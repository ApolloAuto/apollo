load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CANBUS_COPTS = ["-DMODULE_NAME=\\\"canbus\\\""]

apollo_cc_library(
    name = "apollo_canbus_vehicle_demo",
    srcs = [
        "demo_controller.cc",
        "demo_message_manager.cc",
        "demo_vehicle_factory.cc",
        "protocol/brake_command_101.cc",
        "protocol/gear_command_103.cc",
        "protocol/park_command_104.cc",
        "protocol/steering_command_102.cc",
        "protocol/throttle_command_100.cc",
        "protocol/vehicle_mode_command_105.cc",
        "protocol/bms_report_512.cc",
        "protocol/brake_report_501.cc",
        "protocol/gear_report_503.cc",
        "protocol/park_report_504.cc",
        "protocol/steering_report_502.cc",
        "protocol/throttle_report_500.cc",
        "protocol/ultr_sensor_1_507.cc",
        "protocol/ultr_sensor_2_508.cc",
        "protocol/ultr_sensor_3_509.cc",
        "protocol/ultr_sensor_4_510.cc",
        "protocol/ultr_sensor_5_511.cc",
        "protocol/vcu_report_505.cc",
        "protocol/vin_resp1_514.cc",
        "protocol/vin_resp2_515.cc",
        "protocol/vin_resp3_516.cc",
        "protocol/wheelspeed_report_506.cc",
    ],
    hdrs = [
        "demo_controller.h",
        "demo_message_manager.h",
        "demo_vehicle_factory.h",
        "protocol/brake_command_101.h",
        "protocol/gear_command_103.h",
        "protocol/park_command_104.h",
        "protocol/steering_command_102.h",
        "protocol/throttle_command_100.h",
        "protocol/vehicle_mode_command_105.h",
        "protocol/bms_report_512.h",
        "protocol/brake_report_501.h",
        "protocol/gear_report_503.h",
        "protocol/park_report_504.h",
        "protocol/steering_report_502.h",
        "protocol/throttle_report_500.h",
        "protocol/ultr_sensor_1_507.h",
        "protocol/ultr_sensor_2_508.h",
        "protocol/ultr_sensor_3_509.h",
        "protocol/ultr_sensor_4_510.h",
        "protocol/ultr_sensor_5_511.h",
        "protocol/vcu_report_505.h",
        "protocol/vin_resp1_514.h",
        "protocol/vin_resp2_515.h",
        "protocol/vin_resp3_516.h",
        "protocol/wheelspeed_report_506.h",
    ],
    copts = CANBUS_COPTS,
    deps = [
        "//modules/canbus:apollo_canbus",
        "//modules/canbus/proto:canbus_conf_cc_proto",
        "//modules/canbus/proto:vehicle_parameter_cc_proto",
        "//modules/canbus_vehicle/demo/proto:demo_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/status",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/drivers/canbus:apollo_drivers_canbus",
    ],
)

apollo_component(
    name = "libdemo_vehicle_factory_lib.so",
    deps = [":apollo_canbus_vehicle_demo"],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "testdata/**",
    ]),
)

apollo_package()
cpplint()
