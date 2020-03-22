load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "piecewise_jerk_speed_optimizer",
    srcs = ["piecewise_jerk_speed_optimizer.cc"],
    hdrs = ["piecewise_jerk_speed_optimizer.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/proto:error_code_proto",
        "//modules/common/proto:pnc_point_proto",
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
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":piecewise_jerk_speed_nonlinear_ipopt_interface",
        "//modules/common/proto:error_code_proto",
        "//modules/common/proto:pnc_point_proto",
        "//modules/planning/common:speed_profile_generator",
        "//modules/planning/common:st_graph_data",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/trajectory1d:piecewise_jerk_trajectory1d",
        "//modules/planning/math/piecewise_jerk:piecewise_jerk_path_problem",
        "//modules/planning/math/piecewise_jerk:piecewise_jerk_speed_problem",
        "//modules/planning/proto:ipopt_return_status_proto",
        "//modules/planning/tasks/optimizers:speed_optimizer",
    ],
)

cc_library(
    name = "piecewise_jerk_speed_nonlinear_ipopt_interface",
    srcs = ["piecewise_jerk_speed_nonlinear_ipopt_interface.cc"],
    hdrs = ["piecewise_jerk_speed_nonlinear_ipopt_interface.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/proto:pnc_point_proto",
        "//modules/planning/common/path:path_data",
        "//modules/planning/common/trajectory1d:piecewise_jerk_trajectory1d",
        "//third_party:ipopt",
    ],
)

cpplint()
