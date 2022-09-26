load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install", "install_src_files")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

MONITOR_COPTS = ['-DMODULE_NAME=\\"monitor\\"']

cc_binary(
    name = "libmonitor.so",
    linkshared = True,
    linkstatic = True,
    deps = [
        ":monitor_lib",
    ],
)

cc_library(
    name = "monitor_lib",
    srcs = ["monitor.cc"],
    hdrs = ["monitor.h"],
    copts = MONITOR_COPTS,
    visibility = ["//visibility:private"],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/monitor/common:recurrent_runner",
        "//modules/monitor/hardware:esdcan_monitor",
        "//modules/monitor/hardware:gps_monitor",
        "//modules/monitor/hardware:resource_monitor",
        "//modules/monitor/hardware:socket_can_monitor",
        "//modules/monitor/software:camera_monitor",
        "//modules/monitor/software:channel_monitor",
        "//modules/monitor/software:functional_safety_monitor",
        "//modules/monitor/software:latency_monitor",
        "//modules/monitor/software:localization_monitor",
        "//modules/monitor/software:module_monitor",
        "//modules/monitor/software:process_monitor",
        "//modules/monitor/software:recorder_monitor",
        "//modules/monitor/software:summary_monitor",
    ],
    alwayslink = True,
)

filegroup(
    name = "runtime_data",
    srcs = glob([
       "dag/*.dag",
       "launch/*.launch",
    ]),
)

install(
    name = "install",
    library_dest = "monitor/lib",
    data_dest = "monitor",
    targets = [
        ":libmonitor.so",
    ],
    data = [
       ":runtime_data",
        ":cyberfile.xml",
        ":monitor.BUILD",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_monitor_src",
        ":install_monitor_hdrs"
    ],
)

install_src_files(
    name = "install_monitor_src",
    src_dir = ["."],
    dest = "monitor/src",
    filter = "*",
)

install_src_files(
    name = "install_monitor_hdrs",
    src_dir = ["."],
    dest = "monitor/include",
    filter = "*.h",
) 

cpplint()
