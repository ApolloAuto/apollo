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
    name = "conti_radar_message_manager",
    srcs = ["conti_radar_message_manager.cc"],
    hdrs = ["conti_radar_message_manager.h"],
    deps = [
        "//modules/common/util:util_tool",
        "//modules/drivers/canbus:apollo_drivers_canbus",
        "//modules/drivers/radar/conti_radar/protocol:drivers_conti_radar_protocol",
    ],
)

apollo_component(
    name = "libconti_radar.so",
    srcs = ["conti_radar_canbus_component.cc"],
    hdrs = ["conti_radar_canbus_component.h"],
    copts = ['-DMODULE_NAME=\\"conti_radar\\"'],
    deps = [
        ":conti_radar_message_manager",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/drivers/canbus:apollo_drivers_canbus",
        "//modules/drivers/canbus/proto:sensor_canbus_conf_cc_proto",
        "//modules/drivers/radar/conti_radar/protocol:drivers_conti_radar_protocol",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/localization_msgs:pose_cc_proto",
        "@eigen",
    ],
)

apollo_package()
cpplint()
