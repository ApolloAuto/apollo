load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "st_graph_point",
    srcs = ["st_graph_point.cc"],
    hdrs = ["st_graph_point.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/speed:st_point",
    ],
)

cc_library(
    name = "dp_st_cost",
    srcs = ["dp_st_cost.cc"],
    hdrs = ["dp_st_cost.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":st_graph_point",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/planning/common:frame",
        "//modules/planning/common:obstacle",
        "//modules/planning/common/speed:st_boundary",
        "//modules/planning/tasks/utils:st_gap_estimator",
    ],
)

cc_library(
    name = "gridded_path_time_graph",
    srcs = ["gridded_path_time_graph.cc"],
    hdrs = ["gridded_path_time_graph.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":dp_st_cost",
        ":st_graph_point",
        "//cyber",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/status",
        "//modules/planning/common:obstacle",
        "//modules/planning/common:path_decision",
        "//modules/planning/common:st_graph_data",
        "//modules/planning/common/speed:speed_data",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/proto:planning_config_cc_proto",
    ],
)

cc_library(
    name = "path_time_heuristic_optimizer",
    srcs = ["path_time_heuristic_optimizer.cc"],
    hdrs = ["path_time_heuristic_optimizer.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":dp_st_cost",
        ":gridded_path_time_graph",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:st_graph_data",
        "//modules/common_msgs/planning_msgs:planning_internal_cc_proto",
        "//modules/planning/tasks/deciders/speed_bounds_decider:st_boundary_mapper",
        "//modules/planning/tasks/optimizers:speed_optimizer",
    ],
)

cc_test(
    name = "gridded_path_time_graph_test",
    size = "small",
    srcs = ["gridded_path_time_graph_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    deps = [
        ":gridded_path_time_graph",
        "//modules/planning/proto:planning_config_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
