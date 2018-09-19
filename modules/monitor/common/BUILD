load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "recurrent_runner",
    srcs = ["recurrent_runner.cc"],
    hdrs = ["recurrent_runner.h"],
    deps = [
        ":monitor_manager",
        "//framework:cybertron",
        "//modules/common/time",
    ],
)

cc_library(
    name = "monitor_manager",
    srcs = ["monitor_manager.cc"],
    hdrs = ["monitor_manager.h"],
    deps = [
        "//modules/canbus/proto:canbus_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/util",
        "//modules/common/util:map_util",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/monitor/proto:monitor_conf_proto",
        "//modules/monitor/proto:system_status_proto",
    ],
)

cpplint()
