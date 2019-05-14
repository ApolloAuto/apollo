load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "decider",
    srcs = [
        "decider_creep.cc",
    ],
    hdrs = [
        "decider_creep.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":decider_base",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common:st_graph_data",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/math/piecewise_jerk:piecewise_jerk_path_problem",
        "//modules/planning/scenarios/util:scenario_util_lib",
        "//modules/planning/tasks/deciders/speed_bounds_decider",
        "//modules/planning/tasks/deciders/speed_bounds_decider:st_boundary_mapper",
    ],
)

cc_library(
    name = "path_decider_obstacle_utils",
    srcs = [
        "path_decider_obstacle_utils.cc",
    ],
    hdrs = [
        "path_decider_obstacle_utils.h",
    ],
    deps = [
        "//modules/planning/common:frame",
    ],
)

cc_library(
    name = "decider_base",
    srcs = [
        "decider.cc",
    ],
    hdrs = [
        "decider.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/tasks:task",
    ],
)

cpplint()
