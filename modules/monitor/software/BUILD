load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

MONITOR_COPTS = ['-DMODULE_NAME=\\"monitor\\"']

cc_library(
    name = "process_monitor",
    srcs = ["process_monitor.cc"],
    hdrs = ["process_monitor.h"],
    copts = MONITOR_COPTS,
    deps = [
        ":summary_monitor",
        "//modules/dreamview/proto:hmi_mode_cc_proto",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "module_monitor",
    srcs = ["module_monitor.cc"],
    hdrs = ["module_monitor.h"],
    copts = MONITOR_COPTS,
    deps = [
        ":summary_monitor",
        "//cyber",
        "//modules/dreamview/proto:hmi_mode_cc_proto",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
        "@com_google_absl//:absl",
    ],
)

cc_library(
    name = "camera_monitor",
    srcs = ["camera_monitor.cc"],
    hdrs = ["camera_monitor.h"],
    copts = MONITOR_COPTS,
    deps = [
        ":summary_monitor",
        "//cyber",
        "//modules/dreamview/proto:hmi_mode_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
        "@com_google_absl//:absl",
    ],
)

cc_library(
    name = "channel_monitor",
    srcs = ["channel_monitor.cc"],
    hdrs = ["channel_monitor.h"],
    copts = MONITOR_COPTS,
    deps = [
        ":latency_monitor",
        ":summary_monitor",
        "//modules/common/latency_recorder/proto:latency_record_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_detail_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/dreamview/proto:hmi_mode_cc_proto",
        "//modules/common_msgs/sensor_msgs:conti_radar_cc_proto",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/common_msgs/localization_msgs:pose_cc_proto",
        "//modules/common_msgs/planning_msgs:navigation_cc_proto",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_cc_proto",
        "@com_google_absl//:absl",
    ],
)

cc_library(
    name = "localization_monitor",
    srcs = ["localization_monitor.cc"],
    hdrs = ["localization_monitor.h"],
    copts = MONITOR_COPTS,
    deps = [
        ":summary_monitor",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
    ],
)

cc_library(
    name = "functional_safety_monitor",
    srcs = ["functional_safety_monitor.cc"],
    hdrs = ["functional_safety_monitor.h"],
    copts = MONITOR_COPTS,
    deps = [
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
    ],
)

cc_library(
    name = "summary_monitor",
    srcs = ["summary_monitor.cc"],
    hdrs = ["summary_monitor.h"],
    copts = MONITOR_COPTS,
    deps = [
        "//modules/common/adapters:adapter_gflags",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
        "@com_google_absl//:absl",
    ],
)

cc_library(
    name = "recorder_monitor",
    srcs = ["recorder_monitor.cc"],
    hdrs = ["recorder_monitor.h"],
    copts = MONITOR_COPTS,
    deps = [
        ":summary_monitor",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common_msgs/monitor_msgs:smart_recorder_status_cc_proto",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
    ],
)

cc_library(
    name = "latency_monitor",
    srcs = ["latency_monitor.cc"],
    hdrs = ["latency_monitor.h"],
    copts = MONITOR_COPTS,
    deps = [
        ":summary_monitor",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder/proto:latency_record_cc_proto",
        "//modules/monitor/common:monitor_manager",
        "//modules/monitor/common:recurrent_runner",
    ],
)

cpplint()
