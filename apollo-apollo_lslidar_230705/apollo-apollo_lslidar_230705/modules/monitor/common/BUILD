load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

MONITOR_COPTS = ['-DMODULE_NAME=\\"monitor\\"']

cc_library(
    name = "recurrent_runner",
    srcs = ["recurrent_runner.cc"],
    hdrs = ["recurrent_runner.h"],
    copts = MONITOR_COPTS,
    deps = [
        "//cyber",
    ],
)

cc_test(
    name = "recurrent_runner_test",
    size = "small",
    srcs = ["recurrent_runner_test.cc"],
    copts = MONITOR_COPTS,
    deps = [
        ":recurrent_runner",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

cc_library(
    name = "monitor_manager",
    srcs = ["monitor_manager.cc"],
    hdrs = ["monitor_manager.h"],
    copts = MONITOR_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common_msgs/basic_msgs:drive_event_cc_proto",
        "//modules/common/util",
        "//modules/common/util:util_tool",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/hmi:hmi_worker",
        "//modules/dreamview/proto:hmi_config_cc_proto",
        "//modules/dreamview/proto:hmi_mode_cc_proto",
        "//modules/common_msgs/dreamview_msgs:hmi_status_cc_proto",
        "//modules/common_msgs/localization_msgs:pose_cc_proto",
        "//modules/common_msgs/monitor_msgs:system_status_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
