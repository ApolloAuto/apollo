load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "lattice_trajectory1d",
    srcs = ["lattice_trajectory1d.cc"],
    hdrs = ["lattice_trajectory1d.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//cyber",
        "//modules/planning/planning_base/math/curve1d",
    ],
)

cc_library(
    name = "end_condition_sampler",
    srcs = ["end_condition_sampler.cc"],
    hdrs = ["end_condition_sampler.h"],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/lattice/behavior:feasible_region",
        "//modules/planning/planning_base/lattice/behavior:path_time_graph",
        "//modules/planning/planning_base/lattice/behavior:prediction_querier",
        "//modules/planning/planning_base/proto:lattice_structure_cc_proto",
    ],
)

cc_library(
    name = "trajectory1d_generator",
    srcs = ["trajectory1d_generator.cc"],
    hdrs = ["trajectory1d_generator.h"],
    deps = [
        ":end_condition_sampler",
        ":lateral_osqp_optimizer",
        ":lateral_qp_optimizer",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common/trajectory1d:constant_deceleration_trajectory1d",
        "//modules/planning/planning_base/common/trajectory1d:piecewise_acceleration_trajectory1d",
        "//modules/planning/planning_base/common/trajectory1d:piecewise_trajectory1d",
        "//modules/planning/planning_base/common/trajectory1d:standing_still_trajectory1d",
        "//modules/planning/planning_base/lattice/behavior:path_time_graph",
        "//modules/planning/planning_base/lattice/behavior:prediction_querier",
        "//modules/planning/planning_base/lattice/trajectory_generation:lattice_trajectory1d",
        "//modules/planning/planning_base/math/curve1d:quartic_polynomial_curve1d",
        "//modules/planning/planning_base/math/curve1d:quintic_polynomial_curve1d",
        "//modules/planning/planning_base/proto:lattice_structure_cc_proto",
    ],
)

cc_library(
    name = "trajectory_evaluator",
    srcs = ["trajectory_evaluator.cc"],
    hdrs = ["trajectory_evaluator.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/math",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common/trajectory1d:piecewise_acceleration_trajectory1d",
        "//modules/planning/planning_base/constraint_checker:constraint_checker1d",
        "//modules/planning/planning_base/lattice/behavior:path_time_graph",
        "//modules/planning/planning_base/lattice/trajectory_generation:piecewise_braking_trajectory_generator",
        "//modules/planning/planning_base/math/curve1d",
    ],
)

cc_library(
    name = "backup_trajectory_generator",
    srcs = ["backup_trajectory_generator.cc"],
    hdrs = ["backup_trajectory_generator.h"],
    deps = [
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common/trajectory:discretized_trajectory",
        "//modules/planning/planning_base/common/trajectory1d:constant_deceleration_trajectory1d",
        "//modules/planning/planning_base/constraint_checker:collision_checker",
        "//modules/planning/planning_base/lattice/trajectory_generation:trajectory1d_generator",
        "//modules/planning/planning_base/lattice/trajectory_generation:trajectory_combiner",
        "//modules/planning/planning_base/math/curve1d",
    ],
)

cc_library(
    name = "trajectory_combiner",
    srcs = ["trajectory_combiner.cc"],
    hdrs = ["trajectory_combiner.h"],
    deps = [
        "//modules/common/math",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/common/trajectory:discretized_trajectory",
        "//modules/planning/planning_base/math/curve1d",
    ],
)

cc_library(
    name = "piecewise_braking_trajectory_generator",
    srcs = ["piecewise_braking_trajectory_generator.cc"],
    hdrs = ["piecewise_braking_trajectory_generator.h"],
    deps = [
        "//modules/planning/planning_base/common/trajectory1d:piecewise_acceleration_trajectory1d",
        "//modules/planning/planning_base/math/curve1d",
    ],
)

cc_library(
    name = "lateral_qp_optimizer",
    srcs = ["lateral_qp_optimizer.cc"],
    hdrs = ["lateral_qp_optimizer.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/util",
        "//modules/planning/planning_base/common/trajectory1d:piecewise_jerk_trajectory1d",
    ],
)

cc_library(
    name = "lateral_osqp_optimizer",
    srcs = ["lateral_osqp_optimizer.cc"],
    hdrs = ["lateral_osqp_optimizer.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":lateral_qp_optimizer",
        "//cyber",
        "//modules/common/math",
        "//modules/planning/planning_base/common/trajectory1d:piecewise_jerk_trajectory1d",
        "@eigen",
        "@osqp",
    ],
)

apollo_package()
cpplint()
