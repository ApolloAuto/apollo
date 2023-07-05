load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "piecewise_jerk_path_optimizer",
    srcs = ["piecewise_jerk_path_optimizer.cc"],
    hdrs = ["piecewise_jerk_path_optimizer.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/math",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/util",
        "//modules/planning/common:obstacle",
        "//modules/planning/common:path_boundary",
        "//modules/planning/common:path_decision",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/path:discretized_path",
        "//modules/planning/common/path:frenet_frame_path",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/speed:speed_data",
        "//modules/planning/lattice/trajectory_generation:trajectory1d_generator",
        "//modules/planning/math:polynomial_xd",
        "//modules/planning/math/curve1d:polynomial_curve1d",
        "//modules/planning/math/curve1d:quintic_polynomial_curve1d",
        "//modules/planning/math/piecewise_jerk:piecewise_jerk_path_problem",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/reference_line",
        "//modules/planning/tasks/optimizers:path_optimizer",
        "@eigen",
    ],
)

cc_library(
    name = "piecewise_jerk_path_ipopt_solver",
    srcs = ["piecewise_jerk_path_ipopt_solver.cc"],
    hdrs = ["piecewise_jerk_path_ipopt_solver.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "@eigen",
        "@ipopt",
    ],
)

cpplint()
