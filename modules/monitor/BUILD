load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "monitor_lib",
    srcs = ["monitor.cc"],
    hdrs = ["monitor.h"],
    copts = ['-DMODULE_NAME=\\"monitor\\"'],
    deps = [
        "//cyber",
        "//modules/common/util:message_util",
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
)

cc_binary(
    name = "libmonitor.so",
    linkshared = True,
    linkstatic = False,
    deps = [
        ":monitor_lib",
    ],
)

cpplint()
