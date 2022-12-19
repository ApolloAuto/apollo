load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "speed_bounds_decider",
    srcs = ["speed_bounds_decider.cc"],
    hdrs = ["speed_bounds_decider.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":st_boundary_mapper",
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:st_graph_data",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/tasks:task",
        "//modules/planning/tasks/deciders:decider_base",
        "//modules/planning/tasks/deciders/lane_change_decider",
    ],
)

cc_library(
    name = "st_boundary_mapper",
    srcs = [
        "speed_limit_decider.cc",
        "st_boundary_mapper.cc",
    ],
    hdrs = [
        "speed_limit_decider.h",
        "st_boundary_mapper.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/status",
        "//modules/map/pnc_map",
        "//modules/planning/common:dependency_injector",
        "//modules/planning/common:frame",
        "//modules/planning/common:obstacle",
        "//modules/planning/common:path_decision",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:speed_limit",
        "//modules/planning/common/path:discretized_path",
        "//modules/planning/common/path:frenet_frame_path",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed:st_boundary",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/reference_line",
    ],
)

cc_test(
    name = "st_boundary_mapper_test",
    size = "small",
    srcs = ["st_boundary_mapper_test.cc"],
    data = [
        "//modules/planning:planning_testdata",
    ],
    deps = [
        ":st_boundary_mapper",
        "//cyber",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
