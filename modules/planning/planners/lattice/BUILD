load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_plugin")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
    ])
)

apollo_plugin(
    name = "liblattice_planner.so",
    srcs = [
        "lattice_planner.cc",
    ],
    hdrs = [
        "lattice_planner.h",
    ],
    description = ":plugins.xml",
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        ":lattice_planner_base",
    ],
)

apollo_cc_library(
    name = "lattice_planner_base",
    srcs = [
        "behavior/feasible_region.cc",
        "behavior/path_time_graph.cc",
        "behavior/prediction_querier.cc",
        "behavior/collision_checker.cc",
        "trajectory_generation/backup_trajectory_generator.cc",
        "trajectory_generation/end_condition_sampler.cc",
        "trajectory_generation/lateral_osqp_optimizer.cc",
        "trajectory_generation/lateral_qp_optimizer.cc",
        "trajectory_generation/lattice_trajectory1d.cc",
        "trajectory_generation/piecewise_braking_trajectory_generator.cc",
        "trajectory_generation/trajectory1d_generator.cc",
        "trajectory_generation/trajectory_combiner.cc",
        "trajectory_generation/trajectory_evaluator.cc",
    ],
    hdrs = [
        "behavior/feasible_region.h",
        "behavior/path_time_graph.h",
        "behavior/prediction_querier.h",
        "behavior/collision_checker.h",
        "trajectory_generation/backup_trajectory_generator.h",
        "trajectory_generation/end_condition_sampler.h",
        "trajectory_generation/lateral_osqp_optimizer.h",
        "trajectory_generation/lateral_qp_optimizer.h",
        "trajectory_generation/lattice_trajectory1d.h",
        "trajectory_generation/piecewise_braking_trajectory_generator.h",
        "trajectory_generation/trajectory1d_generator.h",
        "trajectory_generation/trajectory_combiner.h",
        "trajectory_generation/trajectory_evaluator.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/planning/planning_interface_base:apollo_planning_planning_interface_base",
    ],
)

apollo_package()
cpplint()
