load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

MONITOR_COPTS = ['-DMODULE_NAME=\\"monitor\\"']

apollo_cc_library(
    name = "apollo_monitor",
    srcs = [
        "common/recurrent_runner.cc",
        "common/monitor_manager.cc",
        "hardware/gps_monitor.cc",
        "hardware/resource_monitor.cc",
        "hardware/esdcan_monitor.cc",
        "hardware/socket_can_monitor.cc",
        "software/process_monitor.cc",
        "software/module_monitor.cc",
        "software/camera_monitor.cc",
        "software/channel_monitor.cc",
        "software/localization_monitor.cc",
        "software/functional_safety_monitor.cc",
        "software/summary_monitor.cc",
        "software/recorder_monitor.cc",
        "software/latency_monitor.cc",
    ],
    hdrs = [
        "common/recurrent_runner.h",
        "common/monitor_manager.h",
        "hardware/gps_monitor.h",
        "hardware/resource_monitor.h",
        "hardware/esdcan_monitor.h",
        "hardware/socket_can_monitor.h",
        "software/process_monitor.h",
        "software/module_monitor.h",
        "software/camera_monitor.h",
        "software/channel_monitor.h",
        "software/localization_monitor.h",
        "software/functional_safety_monitor.h",
        "software/summary_monitor.h",
        "software/recorder_monitor.h",
        "software/latency_monitor.h"
    ],
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:drive_event_cc_proto",
        "//modules/common/util:common_util",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/sensor_msgs:conti_radar_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/common_msgs/localization_msgs:pose_cc_proto",
        "//modules/common_msgs/chassis_msgs:chassis_detail_cc_proto",
        "//modules/common_msgs/monitor_msgs:system_status_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/dreamview_msgs:hmi_mode_cc_proto",
        "//modules/common_msgs/planning_msgs:navigation_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder/proto:latency_record_cc_proto",
        "//modules/common/monitor_log",
        "//modules/common_msgs/dreamview_msgs:hmi_config_cc_proto",
        "//modules/common_msgs/dreamview_msgs:hmi_status_cc_proto",
        "//modules/dreamview/backend/common:dreamview_common",
        "//modules/common_msgs/sensor_msgs:gnss_best_pose_cc_proto",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_cc_proto",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/monitor_msgs:smart_recorder_status_cc_proto",
        "//modules/drivers/canbus:apollo_drivers_canbus",
        "@boost",
        "@com_google_absl//:absl",
        "@com_github_gflags_gflags//:gflags",
    ]
)

apollo_component(
    name = "libmonitor.so",
    srcs = ["monitor.cc"],
    hdrs = ["monitor.h"],
    copts = MONITOR_COPTS,
    visibility = ["//visibility:private"],
    deps = [
        ":apollo_monitor",
        "//cyber",
        "//modules/common/util:util_tool",
    ],
)

apollo_cc_test(
    name = "recurrent_runner_test",
    size = "small",
    srcs = ["common/recurrent_runner_test.cc"],
    copts = MONITOR_COPTS,
    linkstatic = True,
    deps = [
        ":apollo_monitor",
        "@com_google_googletest//:gtest_main",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
       "dag/*.dag",
       "launch/*.launch",
    ]),
)

apollo_package()
cpplint()
