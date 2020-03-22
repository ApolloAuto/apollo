load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "task",
    srcs = ["task.cc"],
    hdrs = ["task.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/common/trajectory:publishable_trajectory",
        "//modules/planning/proto:planning_config_proto",
    ],
)

cc_library(
    name = "task_factory",
    srcs = ["task_factory.cc"],
    hdrs = ["task_factory.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
        "-fopenmp",
    ],
    deps = [
        ":task",
        "//modules/planning/proto:planning_config_proto",
        "//modules/planning/tasks/deciders/creep_decider",
        "//modules/planning/tasks/deciders/lane_change_decider",
        "//modules/planning/tasks/deciders/open_space_decider:open_space_fallback_decider",
        "//modules/planning/tasks/deciders/open_space_decider:open_space_pre_stop_decider",
        "//modules/planning/tasks/deciders/open_space_decider:open_space_roi_decider",
        "//modules/planning/tasks/deciders/path_assessment_decider",
        "//modules/planning/tasks/deciders/path_bounds_decider",
        "//modules/planning/tasks/deciders/path_decider",
        "//modules/planning/tasks/deciders/path_lane_borrow_decider",
        "//modules/planning/tasks/deciders/path_reuse_decider",
        "//modules/planning/tasks/deciders/rss_decider",
        "//modules/planning/tasks/deciders/rule_based_stop_decider",
        "//modules/planning/tasks/deciders/speed_bounds_decider",
        "//modules/planning/tasks/deciders/speed_decider",
        "//modules/planning/tasks/deciders/st_bounds_decider",
        "//modules/planning/tasks/optimizers/open_space_trajectory_generation:open_space_trajectory_provider",
        "//modules/planning/tasks/optimizers/open_space_trajectory_partition",
        "//modules/planning/tasks/optimizers/path_time_heuristic:path_time_heuristic_optimizer",
        "//modules/planning/tasks/optimizers/piecewise_jerk_path:piecewise_jerk_path_optimizer",
        "//modules/planning/tasks/optimizers/piecewise_jerk_speed:piecewise_jerk_speed_nonlinear_optimizer",
        "//modules/planning/tasks/optimizers/piecewise_jerk_speed:piecewise_jerk_speed_optimizer",
    ],
)

cpplint()
