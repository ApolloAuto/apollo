load("//tools:apollo_package.bzl", "apollo_package", "apollo_component", "apollo_cc_library")
load("//tools:cpplint.bzl", "cpplint")

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
    name = "racobit_radar_message_manager",
    srcs = ["racobit_radar_message_manager.cc"],
    hdrs = ["racobit_radar_message_manager.h"],
    deps = [
        "//modules/common/util:util_tool",
        "//modules/drivers/canbus:apollo_drivers_canbus",
        "//modules/common_msgs/sensor_msgs:racobit_radar_cc_proto",
        "//modules/drivers/radar/racobit_radar/protocol:drivers_racobit_radar_protocol",
    ],
)

apollo_component(
    name = "libracobit_radar_canbus_component.so",
    srcs = ["racobit_radar_canbus_component.cc"],
    hdrs = ["racobit_radar_canbus_component.h"],
    copts = ['-DMODULE_NAME=\\"racobit_radar_canbus\\"'],
    deps = [
        ":racobit_radar_message_manager",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/status",
        "//modules/drivers/canbus:apollo_drivers_canbus",
        "//modules/drivers/canbus/proto:sensor_canbus_conf_cc_proto",
        "//modules/drivers/radar/racobit_radar/protocol:drivers_racobit_radar_protocol",
    ],
)

apollo_package()
cpplint()
