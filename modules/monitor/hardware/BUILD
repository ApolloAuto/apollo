load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gps_monitor",
    srcs = ["gps_monitor.cc"],
    hdrs = ["gps_monitor.h"],
    deps = [
        "//cyber",
        "//modules/drivers/gnss/proto:gnss_proto",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
        "//modules/monitor/software:summary_monitor",
    ],
)

cc_library(
    name = "resource_monitor",
    srcs = ["resource_monitor.cc"],
    hdrs = ["resource_monitor.h"],
    deps = [
        "//modules/common/util",
        "//modules/dreamview/proto:hmi_mode_proto",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
        "//modules/monitor/software:summary_monitor",
        "//third_party:boost",
    ],
)

cc_library(
    name = "esdcan_monitor",
    srcs = ["esdcan_monitor.cc"],
    hdrs = ["esdcan_monitor.h"],
    deps = [
        "//modules/common/util",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
        "//modules/monitor/software:summary_monitor",
    ] + select({
        "//tools/platforms:use_esd_can": [
            "//third_party/can_card_library/esd_can",
        ],
        "//conditions:default": [],
    }),
)

cc_library(
    name = "socket_can_monitor",
    srcs = ["socket_can_monitor.cc"],
    hdrs = ["socket_can_monitor.h"],
    deps = [
        "//modules/common/util",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
        "//modules/monitor/software:summary_monitor",
    ],
)

cpplint()
