load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lattice_planner",
    hdrs = [
        "lattice_planner.h",
    ],
    srcs = [
        "lattice_planner.cc",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/vehicle_state",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/planner:planner",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/lattice:lattice_params",
        "//modules/planning/lattice:reference_line_matcher",
        "//modules/planning/lattice:trajectory1d_generator",
        "//modules/planning/lattice:trajectory_evaluator",
        "//modules/planning/lattice:collision_checker",
        "//modules/planning/lattice:lattice_constraint_checker",
        "//modules/planning/lattice:decision_analyzer",
        "//modules/planning/lattice/behavior_decider:behavior_decider",
        "//modules/planning/lattice:lattice_util",
    ],
)

cpplint()
