load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lattice_planner",
    srcs = ["lattice_planner.cc"],
    hdrs = ["lattice_planner.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/planning_base/common:planning_gflags",
        "//modules/planning/planning_base/constraint_checker",
        "//modules/planning/planning_base/constraint_checker:collision_checker",
        "//modules/planning/planning_base/lattice/behavior:path_time_graph",
        "//modules/planning/planning_base/lattice/trajectory_generation:backup_trajectory_generator",
        "//modules/planning/planning_base/lattice/trajectory_generation:lattice_trajectory1d",
        "//modules/planning/planning_base/lattice/trajectory_generation:trajectory1d_generator",
        "//modules/planning/planning_base/lattice/trajectory_generation:trajectory_combiner",
        "//modules/planning/planning_base/lattice/trajectory_generation:trajectory_evaluator",
        "//modules/planning/planning_base/planner",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
    ],
)

apollo_package()
cpplint()
