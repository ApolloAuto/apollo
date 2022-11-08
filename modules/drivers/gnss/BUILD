load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

GNSS_COPTS = ['-DMODULE_NAME=\\"gnss\\"']

cc_binary(
    name = "libgnss_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":gnss_component_lib"],
)

cc_library(
    name = "gnss_component_lib",
    srcs = ["gnss_component.cc"],
    hdrs = ["gnss_component.h"],
    copts = GNSS_COPTS,
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common/monitor_log",
        "//modules/common_msgs/basic_msgs:drive_state_cc_proto",
        "//modules/common_msgs/basic_msgs:vehicle_signal_cc_proto",
        "//modules/drivers/gnss/proto:gnss_status_cc_proto",
        "//modules/common_msgs/sensor_msgs:ins_cc_proto",
        "//modules/drivers/gnss/stream:gnss_stream",
    ],
)

install(
    name = "install",
    data_dest = "drivers/addition_data/gnss",
    library_dest = "drivers/lib/gnss",
    data = [
        ":runtime_data",
    ],
    targets = [
        ":libgnss_component.so",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

cpplint()
