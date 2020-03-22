load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "recurrent_runner",
    srcs = ["recurrent_runner.cc"],
    hdrs = ["recurrent_runner.h"],
    deps = [
        "//cyber",
    ],
)

cc_test(
    name = "recurrent_runner_test",
    size = "small",
    srcs = ["recurrent_runner_test.cc"],
    deps = [
        ":recurrent_runner",
        "@com_google_googletest//:gtest_main",
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
        "//modules/common/proto:drive_event_proto",
        "//modules/common/util",
        "//modules/common/util:map_util",
        "//modules/control/proto:control_proto",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/hmi:hmi_worker",
        "//modules/dreamview/proto:hmi_config_proto",
        "//modules/dreamview/proto:hmi_mode_proto",
        "//modules/dreamview/proto:hmi_status_proto",
        "//modules/localization/proto:pose_proto",
        "//modules/monitor/proto:system_status_proto",
    ],
)

cpplint()
