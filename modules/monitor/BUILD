load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "monitor_lib",
    srcs = ["monitor.cc"],
    hdrs = ["monitor.h"],
    deps = [
        "//modules/common:apollo_app",
        "//modules/common/adapters:adapter_manager",
        "//modules/monitor/common:recurrent_runner",
        "//modules/monitor/hardware/can:can_monitor",
        "//modules/monitor/hardware/gps:gps_monitor",
        "//modules/monitor/proto:system_status_proto",
        "//modules/monitor/reporters:static_info_reporter",
        "//modules/monitor/reporters:vehicle_state_reporter",
        "//modules/monitor/software:process_monitor",
        "//modules/monitor/software:summary_monitor",
        "//modules/monitor/software:topic_monitor",
    ],
)

cc_binary(
    name = "monitor",
    srcs = ["main.cc"],
    deps = [
        ":monitor_lib",
    ],
)

cpplint()
