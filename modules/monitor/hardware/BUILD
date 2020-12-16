load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

MONITOR_COPTS = ['-DMODULE_NAME=\\"monitor\\"']

cc_library(
    name = "gps_monitor",
    srcs = ["gps_monitor.cc"],
    hdrs = ["gps_monitor.h"],
    copts = MONITOR_COPTS,
    deps = [
        "//cyber",
        "//modules/drivers/gnss/proto:gnss_best_pose_cc_proto",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
        "//modules/monitor/software:summary_monitor",
    ],
)

cc_library(
    name = "resource_monitor",
    srcs = ["resource_monitor.cc"],
    hdrs = ["resource_monitor.h"],
    copts = MONITOR_COPTS,
    deps = [
        "//modules/common/util",
        "//modules/dreamview/proto:hmi_mode_cc_proto",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
        "//modules/monitor/software:summary_monitor",
        "@boost",
    ],
)

cc_library(
    name = "esdcan_monitor",
    srcs = ["esdcan_monitor.cc"],
    hdrs = ["esdcan_monitor.h"],
    copts = MONITOR_COPTS,
    deps = [
        "//modules/common/util",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
        "//modules/monitor/software:summary_monitor",
    ] + select({
        "//tools/platform:use_esd_can": [
            "//third_party/can_card_library/esd_can",
        ],
        "//conditions:default": [],
    }),
)

cc_library(
    name = "socket_can_monitor",
    srcs = ["socket_can_monitor.cc"],
    hdrs = ["socket_can_monitor.h"],
    copts = MONITOR_COPTS,
    deps = [
        "//modules/common/util",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
        "//modules/monitor/software:summary_monitor",
    ],
)

cpplint()
