load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_inc_library(
    name = "hardware_checker",
    hdrs = [
        "annotations.h",
        "hardware_checker.h",
    ],
    deps = [
        "//modules/monitor/proto:system_status_proto",
    ],
)

cc_library(
    name = "resource_monitor",
    srcs = ["resource_monitor.cc"],
    hdrs = ["resource_monitor.h"],
    linkopts = [
        "-lboost_filesystem",
    ],
    deps = [
        "//modules/common/util",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
    ],
)

cpplint()
