load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "behavior_decider",
    srcs = [
        "behavior_decider.cc",
    ],
    hdrs = [
        "behavior_decider.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/common/math:box2d",
        "//modules/common/proto:common_proto",
        "//modules/common/proto:pnc_point_proto",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/lattice/behavior_decider:condition_filter",
        "//modules/planning/lattice/behavior_decider:path_time_neighborhood",
        "//modules/planning/lattice/behavior_decider:scenario_manager",
        "//modules/planning/proto:lattice_sampling_config_proto",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/reference_line",
    ],
)

cc_library(
    name = "scenario_manager",
    srcs = [
        "scenario_manager.cc",
    ],
    hdrs = [
        "scenario_manager.h",
    ],
    deps = [
        ":scenarios",
        "//modules/common/proto:common_proto",
        "//modules/common/proto:pnc_point_proto",
        "//modules/planning/common:frame",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/proto:lattice_sampling_config_proto",
    ],
)

cc_library(
    name = "scenarios",
    srcs = [
        "ego_vehicle_scenario.cc",
        "signal_light_scenario.cc",
    ],
    hdrs = [
        "ego_vehicle_scenario.h",
        "scenario.h",
        "signal_light_scenario.h",
    ],
    deps = [
        "//modules/common/adapters:adapter_manager",
        "//modules/common/proto:common_proto",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/util:map_util",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/proto:lattice_structure_proto",
    ],
)

cc_library(
    name = "feasible_region",
    srcs = [
        "feasible_region.cc",
    ],
    hdrs = [
        "feasible_region.h",
    ],
    deps = [
        "//modules/common/proto:pnc_point_proto",
        "//modules/planning/common:planning_gflags",
    ],
)

cc_library(
    name = "path_time_neighborhood",
    srcs = [
        "path_time_neighborhood.cc",
    ],
    hdrs = [
        "path_time_neighborhood.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common/math:linear_interpolation",
        "//modules/common/proto:pnc_point_proto",
        "//modules/planning/common:frame",
        "//modules/planning/common:obstacle",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/lattice/util:reference_line_matcher",
        "//modules/planning/proto:lattice_structure_proto",
        "//modules/planning/reference_line",
        "@glog",
    ],
)

cc_library(
    name = "condition_filter",
    srcs = [
        "condition_filter.cc",
    ],
    hdrs = [
        "condition_filter.h",
    ],
    deps = [
        "//modules/common/math:linear_interpolation",
        "//modules/planning/lattice/behavior_decider:feasible_region",
        "//modules/planning/lattice/behavior_decider:path_time_neighborhood",
        "//modules/planning/proto:lattice_structure_proto",
        "//modules/planning/proto:planning_proto",
    ],
)

cpplint()
