load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "navi_task",
    srcs = ["navi_task.cc"],
    hdrs = ["navi_task.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/proto:planning_config_cc_proto",
    ],
)

cc_library(
    name = "navi_path_decider",
    srcs = ["navi_path_decider.cc"],
    hdrs = ["navi_path_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":navi_task",
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/navi/decider:navi_obstacle_decider",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/tasks:task",
    ],
)

cc_library(
    name = "navi_speed_decider",
    srcs = [
        "navi_speed_decider.cc",
        "navi_speed_ts_graph.cc",
    ],
    hdrs = [
        "navi_speed_decider.h",
        "navi_speed_ts_graph.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        ":navi_task",
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/navi/decider:navi_obstacle_decider",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/tasks:task",
    ],
)

cc_library(
    name = "navi_obstacle_decider",
    srcs = ["navi_obstacle_decider.cc"],
    hdrs = ["navi_obstacle_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":navi_task",
        "//cyber",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/planning/common:frame",
        "//modules/planning/common:obstacle",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/tasks:task",
    ],
)

cc_test(
    name = "navi_decider_test",
    size = "small",
    srcs = [
        "navi_obstacle_decider_test.cc",
        "navi_path_decider_test.cc",
        "navi_speed_decider_test.cc",
        "navi_speed_ts_graph_test.cc",
    ],
    data = ["//modules/planning:planning_testdata"],
    deps = [
        ":navi_obstacle_decider",
        ":navi_path_decider",
        ":navi_speed_decider",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
