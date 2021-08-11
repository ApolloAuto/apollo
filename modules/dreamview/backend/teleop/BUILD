load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "copts_if_teleop", "if_teleop")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "teleop",
    srcs = if_teleop(["teleop.cc"]),
    hdrs = if_teleop(["teleop.h"]),
    copts = ['-DMODULE_NAME=\\"dreamview\\"'] + copts_if_teleop(),
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/proto:drive_event_cc_proto",
        "//modules/common/util",
        "//modules/dreamview/backend/common:dreamview_gflags",
        "//modules/dreamview/backend/handlers:websocket_handler",
        "//modules/planning/proto:planning_cc_proto",
        "//modules/planning/proto:pad_msg_cc_proto",
        "@com_github_nlohmann_json//:json",
    ] + if_teleop([
        "//modules/teleop/modem/proto:modem_info_cc_proto",
        "//modules/teleop/daemon/proto:daemon_cmd_cc_proto",
        "//modules/teleop/daemon/proto:daemon_rpt_cc_proto",
    ]),
)

cc_test(
    name = "teleop_test",
    size = "small",
    srcs = if_teleop(
        ["teleop_test.cc"],
    ),
    data = [
        "//modules/dreamview/backend/testdata",
    ],
    deps = [
        ":teleop",
        "@boost",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
