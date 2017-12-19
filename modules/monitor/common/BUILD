load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "recurrent_runner",
    srcs = ["recurrent_runner.cc"],
    hdrs = ["recurrent_runner.h"],
    deps = [
        "//modules/common:log",
        "//modules/common/time",
    ],
)

cc_library(
    name = "monitor_manager",
    srcs = ["monitor_manager.cc"],
    hdrs = ["monitor_manager.h"],
    deps = [
        "//modules/common/monitor_log",
        "//modules/common/util",
        "//modules/common/util:map_util",
        "//modules/monitor/proto:monitor_conf_proto",
        "//modules/monitor/proto:system_status_proto",
    ],
)

cpplint()
