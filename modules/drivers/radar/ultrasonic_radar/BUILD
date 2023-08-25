load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_component", "apollo_cc_library")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

apollo_cc_library(
    name = "ultrasonic_radar_message_manager",
    srcs = ["ultrasonic_radar_message_manager.cc"],
    hdrs = ["ultrasonic_radar_message_manager.h"],
    deps = [
        "//modules/common_msgs/basic_msgs:header_cc_proto",
        "//modules/common/util:util_tool",
        "//modules/drivers/canbus:apollo_drivers_canbus",
        "//modules/common_msgs/sensor_msgs:ultrasonic_radar_cc_proto",
    ],
)

apollo_cc_library(
    name = "ultrasonic_radar_canbus_lib",
    srcs = ["ultrasonic_radar_canbus.cc"],
    hdrs = ["ultrasonic_radar_canbus.h"],
    deps = [
        ":ultrasonic_radar_message_manager",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/drivers/canbus:apollo_drivers_canbus",
        "//modules/drivers/canbus/proto:sensor_canbus_conf_cc_proto",
        "//modules/drivers/radar/ultrasonic_radar/proto:ultrasonic_radar_conf_cc_proto",
    ],
)


apollo_component(
    name = "libultrasonic_radar_canbus_component.so",
    srcs = ["ultrasonic_radar_canbus_component.cc"],
    hdrs = ["ultrasonic_radar_canbus_component.h"],
    copts = ['-DMODULE_NAME=\\"ultrasonic_radar_canbus\\"'],
    deps = [
        ":ultrasonic_radar_canbus_lib",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common_msgs/sensor_msgs:ultrasonic_radar_cc_proto",
    ],
)

apollo_package()
cpplint()
