load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "process_monitor",
    srcs = ["process_monitor.cc"],
    hdrs = ["process_monitor.h"],
    deps = [
        "//external:gflags",
        "//modules/common/util:string_util",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
    ],
)

cc_library(
    name = "topic_monitor",
    srcs = ["topic_monitor.cc"],
    hdrs = ["topic_monitor.h"],
    deps = [
        "//modules/common/adapters:adapter_manager",
        "//modules/common/util:string_util",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
    ],
)

cc_library(
    name = "safety_manager",
    srcs = ["safety_manager.cc"],
    hdrs = ["safety_manager.h"],
    deps = [
        "//modules/common/adapters:adapter_manager",
        "//modules/common/kv_db",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/proto:hmi_config_proto",
        "//modules/monitor/common:monitor_manager",
    ],
)

cc_library(
    name = "summary_monitor",
    srcs = ["summary_monitor.cc"],
    hdrs = ["summary_monitor.h"],
    deps = [
        ":safety_manager",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/util:string_util",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
    ],
)

cpplint()
