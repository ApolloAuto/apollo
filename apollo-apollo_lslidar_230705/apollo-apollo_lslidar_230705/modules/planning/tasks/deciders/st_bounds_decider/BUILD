load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "st_bounds_decider",
    srcs = ["st_bounds_decider.cc"],
    hdrs = ["st_bounds_decider.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":st_driving_limits",
        ":st_guide_line",
        ":st_obstacles_processor",
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:st_graph_data",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/tasks:task",
        "//modules/planning/tasks/deciders:decider_base",
    ],
)

cc_library(
    name = "st_guide_line",
    srcs = ["st_guide_line.cc"],
    hdrs = ["st_guide_line.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/status",
        "//modules/map/pnc_map",
        "//modules/common_msgs/map_msgs:map_cc_proto",
        "//modules/planning/common:frame",
        "//modules/planning/common:obstacle",
        "//modules/planning/common:path_decision",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:speed_limit",
        "//modules/planning/common/path:discretized_path",
        "//modules/planning/common/path:frenet_frame_path",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/common/speed:st_boundary",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/reference_line",
    ],
)

cc_library(
    name = "st_driving_limits",
    srcs = ["st_driving_limits.cc"],
    hdrs = ["st_driving_limits.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common/status",
        "//modules/map/pnc_map",
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
        "//modules/planning/proto:planning_config_cc_proto",
        "//modules/planning/reference_line",
    ],
)

cc_library(
    name = "st_obstacles_processor",
    srcs = ["st_obstacles_processor.cc"],
    hdrs = ["st_obstacles_processor.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/status",
        "//modules/map/pnc_map",
        "//modules/planning/common:frame",
        "//modules/planning/common:history",
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

cpplint()
