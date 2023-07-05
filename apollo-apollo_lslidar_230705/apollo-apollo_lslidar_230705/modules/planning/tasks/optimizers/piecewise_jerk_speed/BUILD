load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "piecewise_jerk_speed_optimizer",
    srcs = ["piecewise_jerk_speed_optimizer.cc"],
    hdrs = ["piecewise_jerk_speed_optimizer.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/planning/common:speed_profile_generator",
        "//modules/planning/common:st_graph_data",
        "//modules/planning/math/piecewise_jerk:piecewise_jerk_speed_problem",
        "//modules/planning/tasks/optimizers:speed_optimizer",
    ],
)

cc_library(
    name = "piecewise_jerk_speed_nonlinear_optimizer",
    srcs = ["piecewise_jerk_speed_nonlinear_optimizer.cc"],
    hdrs = ["piecewise_jerk_speed_nonlinear_optimizer.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":piecewise_jerk_speed_nonlinear_ipopt_interface",
        "//modules/common_msgs/basic_msgs:error_code_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/util",
        "//modules/planning/common:speed_profile_generator",
        "//modules/planning/common:st_graph_data",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/trajectory1d:piecewise_jerk_trajectory1d",
        "//modules/planning/math/piecewise_jerk:piecewise_jerk_path_problem",
        "//modules/planning/math/piecewise_jerk:piecewise_jerk_speed_problem",
        "//modules/planning/proto:ipopt_return_status_cc_proto",
        "//modules/planning/tasks/optimizers:speed_optimizer",
    ],
)

cc_library(
    name = "piecewise_jerk_speed_nonlinear_ipopt_interface",
    srcs = ["piecewise_jerk_speed_nonlinear_ipopt_interface.cc"],
    hdrs = ["piecewise_jerk_speed_nonlinear_ipopt_interface.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/trajectory1d:piecewise_jerk_trajectory1d",
        "@ipopt",
    ],
)

cpplint()
