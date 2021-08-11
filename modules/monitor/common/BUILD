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
)

cc_library(
    name = "monitor_manager",
    srcs = ["monitor_manager.cc"],
    hdrs = ["monitor_manager.h"],
    copts = MONITOR_COPTS,
    deps = [
        "//cyber/common:file",
        "//cyber/common:macros",
        "//modules/canbus/proto:chassis_cc_proto",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/proto:drive_event_cc_proto",
        "//modules/common/util",
        "//modules/common/util:map_util",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/hmi:hmi_worker",
        "//modules/dreamview/proto:hmi_config_cc_proto",
        "//modules/dreamview/proto:hmi_mode_cc_proto",
        "//modules/dreamview/proto:hmi_status_cc_proto",
        "//modules/localization/proto:pose_cc_proto",
        "//modules/monitor/proto:system_status_cc_proto",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
