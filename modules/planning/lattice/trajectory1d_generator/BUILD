load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

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
        "//modules/planning/lattice/util:lattice_util",
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
        "//modules/planning/lattice/trajectory1d:lattice_trajectory1d",
        "//modules/planning/math/curve1d:curve1d",
        "//modules/planning/math/curve1d:quartic_polynomial_curve1d",
        "//modules/planning/math/curve1d:quintic_polynomial_curve1d",
        "//modules/planning/lattice/trajectory1d_generator:end_condition_sampler",
        "//modules/planning/proto:lattice_sampling_config_proto",
        "//modules/planning/proto:lattice_structure_proto",
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
        "//modules/planning/lattice/util:lattice_util",
        "//modules/planning/proto:lattice_sampling_config_proto",
    ],
)

cpplint()
