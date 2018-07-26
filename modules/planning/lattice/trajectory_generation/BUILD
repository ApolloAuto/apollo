load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "end_condition_sampler",
    srcs = [
        "end_condition_sampler.cc",
    ],
    hdrs = [
        "end_condition_sampler.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common/configs:vehicle_config_helper",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/lattice/behavior:feasible_region",
        "//modules/planning/lattice/behavior:path_time_graph",
        "//modules/planning/lattice/behavior:prediction_querier",
        "//modules/planning/proto:lattice_structure_proto",
    ],
)

cc_library(
    name = "trajectory1d_generator",
    srcs = [
        "trajectory1d_generator.cc",
    ],
    hdrs = [
        "trajectory1d_generator.h",
    ],
    deps = [
        "//modules/common",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/lattice/behavior:path_time_graph",
        "//modules/planning/lattice/behavior:prediction_querier",
        "//modules/planning/lattice/trajectory1d:constant_deceleration_trajectory1d",
        "//modules/planning/lattice/trajectory1d:lattice_trajectory1d",
        "//modules/planning/lattice/trajectory1d:piecewise_acceleration_trajectory1d",
        "//modules/planning/lattice/trajectory1d:piecewise_trajectory1d",
        "//modules/planning/lattice/trajectory1d:standing_still_trajectory1d",
        "//modules/planning/lattice/trajectory_generation:end_condition_sampler",
        "//modules/planning/math/curve1d:quartic_polynomial_curve1d",
        "//modules/planning/math/curve1d:quintic_polynomial_curve1d",
        "//modules/planning/proto:lattice_sampling_config_proto",
        "//modules/planning/proto:lattice_structure_proto",
        "//modules/planning/lattice/trajectory_generation:lateral_trajectory_optimizer_interface",
        "//modules/planning/lattice/trajectory_generation:lateral_trajectory_optimizer",
    ],
)

cc_library(
    name = "trajectory_evaluator",
    srcs = [
        "trajectory_evaluator.cc",
    ],
    hdrs = [
        "trajectory_evaluator.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common/math:path_matcher",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/constraint_checker:constraint_checker1d",
        "//modules/planning/lattice/behavior:path_time_graph",
        "//modules/planning/lattice/trajectory1d:piecewise_acceleration_trajectory1d",
        "//modules/planning/lattice/trajectory_generation:piecewise_braking_trajectory_generator",
        "//modules/planning/math/curve1d",
        "//modules/planning/proto:lattice_sampling_config_proto",
    ],
)

cc_library(
    name = "backup_trajectory_generator",
    srcs = [
        "backup_trajectory_generator.cc",
    ],
    hdrs = [
        "backup_trajectory_generator.h",
    ],
    deps = [
        "//modules/common",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/constraint_checker:collision_checker",
        "//modules/planning/lattice/trajectory1d:constant_deceleration_trajectory1d",
        "//modules/planning/lattice/trajectory_generation:trajectory1d_generator",
        "//modules/planning/lattice/trajectory_generation:trajectory_combiner",
        "//modules/planning/math/curve1d",
    ],
)

cc_library(
    name = "trajectory_combiner",
    srcs = [
        "trajectory_combiner.cc",
    ],
    hdrs = [
        "trajectory_combiner.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common/math:cartesian_frenet_conversion",
        "//modules/common/math:path_matcher",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/math/curve1d",
    ],
)

cc_library(
    name = "piecewise_braking_trajectory_generator",
    srcs = [
        "piecewise_braking_trajectory_generator.cc",
    ],
    hdrs = [
        "piecewise_braking_trajectory_generator.h",
    ],
    deps = [
        "//modules/planning/lattice/trajectory1d:piecewise_acceleration_trajectory1d",
        "//modules/common",
        "//modules/planning/math/curve1d",
    ],
)

cc_library(
    name = "lateral_trajectory_optimizer_interface",
    srcs = [
        "lateral_trajectory_optimizer_interface.cc",
    ],
    hdrs = [
        "lateral_trajectory_optimizer_interface.h",
    ],
    deps = [
        "//modules/planning/lattice/trajectory1d:constant_jerk_trajectory1d",
        "@ipopt",
    ],
)

cc_library(
    name = "lateral_trajectory_optimizer",
    srcs = [
        "lateral_trajectory_optimizer.cc",
    ],
    hdrs = [
        "lateral_trajectory_optimizer.h",
    ],
    deps = [
        "//modules/planning/lattice/behavior:path_time_graph",
        "//modules/planning/lattice/trajectory_generation:lateral_trajectory_optimizer_interface",
        "//modules/planning/lattice/trajectory1d:piecewise_jerk_trajectory1d",
        "//modules/planning/common:planning_gflags",
        "@ipopt",
    ],
)

cpplint()
