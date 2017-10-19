load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "standing_still_trajectory1d",
    hdrs = [
        "standing_still_trajectory1d.h",
    ],
    srcs = [
        "standing_still_trajectory1d.cc",
    ],
    deps = [
        "//modules/planning/math/curve1d:curve1d",
    ],
)

cc_library(
    name = "piecewise_acceleration_trajectory1d",
    hdrs = [
        "piecewise_acceleration_trajectory1d.h",
    ],
    srcs = [
        "piecewise_acceleration_trajectory1d.cc",
    ],
    deps = [
        "//modules/common:common",
        "//modules/common/math:linear_interpolation",
        "//modules/planning/math/curve1d:curve1d",
    ],
)

cc_library(
    name = "constant_deceleration_trajectory1d",
    hdrs = [
        "constant_deceleration_trajectory1d.h",
    ],
    srcs = [
        "constant_deceleration_trajectory1d.cc",
    ],
    deps = [
        "//modules/common:common",
        "//modules/planning/math/curve1d:curve1d",
    ],
)

cc_library(
    name = "piecewise_trajectory1d",
    hdrs = [
        "piecewise_trajectory1d.h",
    ],
    srcs = [
        "piecewise_trajectory1d.cc",
    ],
    deps = [
        "//modules/common:common",
        "//modules/common/math:linear_interpolation",
        "//modules/planning/math/curve1d:curve1d",
    ],
)

cc_library(
    name = "planning_target",
    hdrs = [
        "planning_target.h",
    ],
    srcs = [
        "planning_target.cc",
    ],
    deps = [
    ],
)

cc_library(
    name = "end_condition_sampler",
    hdrs = [
        "end_condition_sampler.h",
    ],
    srcs = [
        "end_condition_sampler.cc",
    ],
    deps = [
        "//modules/common:common",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/lattice:lattice_params",
    ],
)

cc_library(
    name = "trajectory_evaluator",
    hdrs = [
        "trajectory_evaluator.h",
    ],
    srcs = [
        "trajectory_evaluator.cc",
    ],
    deps = [
        "//modules/common:common",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/math/curve1d:curve1d",
        "//modules/planning/lattice:lattice_params",
        "//modules/planning/lattice:lattice_constraint_checker",
        "//modules/planning/proto:lattice_sampling_config_proto",
        "//modules/planning/lattice:planning_target",
        "//modules/planning/lattice:lattice_params",
        "//modules/planning/lattice:lattice_constraint_checker",
        "//modules/planning/proto:lattice_sampling_config_proto",
    ],
)

cc_library(
    name = "trajectory1d_generator",
    hdrs = [
        "trajectory1d_generator.h",
    ],
    srcs = [
        "trajectory1d_generator.cc",
    ],
    deps = [
        "//modules/common:common",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/lattice:standing_still_trajectory1d",
        "//modules/planning/lattice:constant_deceleration_trajectory1d",
        "//modules/planning/math/curve1d:curve1d",
        "//modules/planning/math/curve1d:quartic_polynomial_curve1d",
        "//modules/planning/math/curve1d:quintic_polynomial_curve1d",
        "//modules/planning/lattice:end_condition_sampler",
        "//modules/planning/proto:lattice_sampling_config_proto",
    ],
)

cc_library(
    name = "collision_checker",
    srcs = [
        "collision_checker.cc",
    ],
    hdrs = [
        "collision_checker.h",
    ],
    deps = [
        "//modules/common/math:box2d",
        "//modules/planning/common:frame",
        "//modules/planning/common:obstacle",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/lattice:lattice_params",
        "//modules/prediction/proto:prediction_proto",
        "//modules/common/configs:vehicle_config_helper",
    ],
)

cc_library(
    name = "lattice_constraint_checker",
    srcs = [
        "lattice_constraint_checker.cc",
    ],
    hdrs = [
        "lattice_constraint_checker.h",
    ],
    deps = [
        "//modules/planning/math/curve1d:curve1d",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/common:frame",
    ],
)

cc_library(
    name = "decision_analyzer",
    srcs = [
        "decision_analyzer.cc",
    ],
    hdrs = [
        "decision_analyzer.h",
    ],
    deps = [
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/lattice:planning_target",
        "//modules/common/proto:pnc_point_proto",
        "//external:gflags",
    ],
)

cc_library(
    name = "reference_line_matcher",
    hdrs = [
        "reference_line_matcher.h",
    ],
    srcs = [
        "reference_line_matcher.cc",
    ],
    deps = [
        "//modules/common:common",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/math:hermite_spline",
        "//modules/common/proto:pnc_point_proto",
        "//modules/planning/common:planning_common",
        "@glog//:glog",
    ],
)

cc_library(
    name = "lattice_util",
    hdrs = [
        "lattice_util.h",
    ],
    srcs = [
        "lattice_util.cc",
    ],
    deps = [
        "//modules/common/proto:pnc_point_proto",
        "//modules/planning/reference_line:reference_line",
    ],
)

cc_library(
    name = "lattice_params",
    hdrs = ["lattice_params.h"],
    deps = [],
)

cpplint()
