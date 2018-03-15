load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lattice_planner",
    srcs = [
        "lattice_planner.cc",
    ],
    hdrs = [
        "lattice_planner.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/math:path_matcher",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/constraint_checker",
        "//modules/planning/constraint_checker:collision_checker",
        "//modules/planning/lattice/behavior:behavior_decider",
        "//modules/planning/lattice/behavior:path_time_graph",
        "//modules/planning/lattice/trajectory1d:lattice_trajectory1d",
        "//modules/planning/lattice/trajectory_generation:backup_trajectory_generator",
        "//modules/planning/lattice/trajectory_generation:trajectory1d_generator",
        "//modules/planning/lattice/trajectory_generation:trajectory_combiner",
        "//modules/planning/lattice/trajectory_generation:trajectory_evaluator",
        "//modules/planning/planner",
        "//modules/planning/proto:planning_proto",
    ],
)

cpplint()
