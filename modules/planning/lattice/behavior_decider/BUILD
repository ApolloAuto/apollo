load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "behavior_decider",
    hdrs = [
        "behavior_decider.h",
    ],
    srcs = [
        "behavior_decider.cc",
    ],
    deps = [
        "//modules/common/math:box2d",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:lattice_sampling_config_proto",
        "//modules/planning/lattice/util:lattice_util",
        "//modules/planning/lattice/behavior_decider:scenario_manager",
        "//modules/planning/reference_line",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/proto:common_proto",
        "//external:gflags",
    ],
)

cc_library(
    name = "scenario_manager",
    hdrs = [
        "scenario_manager.h",
    ],
    srcs = [
        "scenario_manager.cc",
    ],
    deps = [
    	":adc_master_scenario",
        "//modules/planning/proto:lattice_sampling_config_proto",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/proto:common_proto",
        "//modules/planning/common:frame",
    ],
)

cc_library(
    name = "adc_master_scenario",
    hdrs = [
        "scenario.h",
        "adc_master_scenario.h",
    ],
    srcs = [
        "adc_master_scenario.cc",
    ],
    deps = [
        "//modules/planning/proto:lattice_structure_proto",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/proto:common_proto",
        "//modules/planning/common:frame",
    ],
)

cc_library(
    name = "feasible_region",
    hdrs = [
        "feasible_region.h",
    ],
    srcs = [
        "feasible_region.cc",
    ],
    deps = [
        "//modules/planning/common:planning_gflags",
        "//modules/planning/lattice/util:lattice_params",
    ],
)

cc_library(
    name = "adc_neighborhood",
    hdrs = [
        "adc_neighborhood.h",
    ],
    srcs = [
        "adc_neighborhood.cc",
    ],
    deps = [
        "//modules/common:common",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/reference_line:reference_line",
        "//modules/common/proto:pnc_point_proto",
        "//modules/planning/proto:lattice_structure_proto",
        "//modules/planning/common:frame",
        "//modules/planning/common:obstacle",
        "//modules/planning/lattice/util:util",
        "//modules/planning/lattice/util:lattice_params",
        "//modules/planning/lattice/util:reference_line_matcher",
        "@glog//:glog",
    ],
)

cc_library(
    name = "condition_filter",
    hdrs = [
        "condition_filter.h",
    ],
    srcs = [
        "condition_filter.cc",
    ],
    deps = [
        "//modules/planning/lattice/behavior_decider:adc_neighborhood",
        "//modules/planning/lattice/behavior_decider:feasible_region",
        "//modules/planning/proto:lattice_structure_proto",
        "//modules/common/math:linear_interpolation",
    ],
)

cpplint()
