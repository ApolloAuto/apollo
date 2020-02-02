load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "process_monitor",
    srcs = ["process_monitor.cc"],
    hdrs = ["process_monitor.h"],
    deps = [
        ":summary_monitor",
        "//modules/dreamview/proto:hmi_mode_proto",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "channel_monitor",
    srcs = ["channel_monitor.cc"],
    hdrs = ["channel_monitor.h"],
    deps = [
        ":latency_monitor",
        ":summary_monitor",
        "//modules/common/latency_recorder/proto:latency_record_proto",
        "//modules/control/proto:control_proto",
        "//modules/dreamview/proto:hmi_mode_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/map/relative_map/proto:navigation_proto",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/prediction/proto:prediction_proto",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "localization_monitor",
    srcs = ["localization_monitor.cc"],
    hdrs = ["localization_monitor.h"],
    deps = [
        ":summary_monitor",
        "//modules/common/adapters:adapter_gflags",
        "//modules/localization/proto:localization_proto",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
    ],
)

cc_library(
    name = "functional_safety_monitor",
    srcs = ["functional_safety_monitor.cc"],
    hdrs = ["functional_safety_monitor.h"],
    deps = [
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
    ],
)

cc_library(
    name = "summary_monitor",
    srcs = ["summary_monitor.cc"],
    hdrs = ["summary_monitor.h"],
    deps = [
        "//modules/common/adapters:adapter_gflags",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "recorder_monitor",
    srcs = ["recorder_monitor.cc"],
    hdrs = ["recorder_monitor.h"],
    deps = [
        ":summary_monitor",
        "//modules/common/adapters:adapter_gflags",
        "//modules/data/tools/smart_recorder/proto:smart_recorder_status_proto",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
    ],
)

cc_library(
    name = "latency_monitor",
    srcs = ["latency_monitor.cc"],
    hdrs = ["latency_monitor.h"],
    deps = [
        ":summary_monitor",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder/proto:latency_record_proto",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
    ],
)

cpplint()
