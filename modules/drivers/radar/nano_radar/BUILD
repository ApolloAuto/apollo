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
    name = "nano_radar_message_manager",
    srcs = ["nano_radar_message_manager.cc"],
    hdrs = ["nano_radar_message_manager.h"],
    deps = [
        "//modules/common/util:message_util",
        "//modules/drivers/canbus:apollo_drivers_canbus",
        "//modules/drivers/radar/nano_radar/protocol:drivers_nano_radar_protocol",
    ],
)

apollo_component(
    name = "libnano_radar.so",
    srcs = ["nano_radar_canbus_component.cc"],
    hdrs = ["nano_radar_canbus_component.h"],
    copts = ['-DMODULE_NAME=\\"nano_radar\\"'],
    deps = [
        ":nano_radar_message_manager",
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/drivers/canbus:apollo_drivers_canbus",
        "//modules/drivers/canbus/proto:sensor_canbus_conf_cc_proto",
        "//modules/drivers/radar/nano_radar/protocol:drivers_nano_radar_protocol",
        "@eigen",
    ],
)

apollo_package()
cpplint()
