load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "gps_monitor",
    srcs = ["gps_monitor.cc"],
    hdrs = ["gps_monitor.h"],
    deps = [
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/monitor/common:monitor_interface",
    ],
)

cpplint()
