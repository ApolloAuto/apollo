load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "process_monitor",
    srcs = ["process_monitor.cc"],
    hdrs = ["process_monitor.h"],
    deps = [
        "//external:gflags",
        "//modules/common/util",
        "//modules/monitor/common:monitor_interface",
        "//modules/monitor/proto:module_monitor_conf_proto",
    ],
)

cpplint()
