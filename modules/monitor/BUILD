load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libmonitor.so",
    srcs = [
        "monitor.cc",
        "monitor.h",
    ],
    deps = [
        "//framework:cybertron",
        "//modules/common/util:message_util",
        "//modules/monitor/common:recurrent_runner",
        "//modules/monitor/hardware/gps:gps_monitor",
        "//modules/monitor/hardware:resource_monitor",
        "//modules/monitor/proto:system_status_proto",
        "//modules/monitor/reporters:static_info_reporter",
        "//modules/monitor/reporters:vehicle_state_reporter",
        "//modules/monitor/software:localization_monitor",
        "//modules/monitor/software:process_monitor",
        "//modules/monitor/software:summary_monitor",
        "//modules/monitor/software:topic_monitor",
    ],
    copts = ['-DMODULE_NAME=\\"monitor\\"'],
    linkshared = True,
)

cpplint()
