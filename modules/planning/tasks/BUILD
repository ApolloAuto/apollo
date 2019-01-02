load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "task",
    srcs = [
        "task.cc",
    ],
    hdrs = [
        "task.h",
    ],
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
    srcs = [
        "task_factory.cc",
    ],
    hdrs = [
        "task_factory.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":task",
        "//modules/planning/proto:planning_config_proto",
        "//modules/planning/tasks/deciders:decider",
        "//modules/planning/tasks/optimizers/dp_poly_path:dp_poly_path_optimizer",
        "//modules/planning/tasks/optimizers/dp_st_speed:dp_st_speed_optimizer",
        "//modules/planning/tasks/optimizers/path_decider",
        "//modules/planning/tasks/optimizers/proceed_with_caution_speed:proceed_with_caution_speed_generator",
        "//modules/planning/tasks/optimizers/qp_piecewise_jerk_path:qp_piecewise_jerk_path_optimizer",
        "//modules/planning/tasks/optimizers/qp_spline_path",
        "//modules/planning/tasks/optimizers/qp_spline_st_speed:qp_spline_st_speed_optimizer",
        "//modules/planning/tasks/optimizers/speed_decider",
        "//modules/planning/tasks/rss:decider_rss",
    ],
)

cpplint()
