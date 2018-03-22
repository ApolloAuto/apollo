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
        "//modules/common/math:geometry",
        "//modules/common/proto:common_proto",
        "//modules/common/proto:pnc_point_proto",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/lattice/behavior:path_time_graph",
        "//modules/planning/lattice/behavior:prediction_querier",
        "//modules/planning/lattice/behavior:scenario_manager",
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
        "//modules/common/configs:vehicle_config_helper",
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
    name = "path_time_graph",
    srcs = [
        "path_time_graph.cc",
    ],
    hdrs = [
        "path_time_graph.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common/math:linear_interpolation",
        "//modules/common/math:path_matcher",
        "//modules/common/proto:pnc_point_proto",
        "//modules/planning/common:frame",
        "//modules/planning/common:obstacle",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto:lattice_structure_proto",
        "//modules/planning/reference_line",
        "@glog",
    ],
)

cc_library(
    name = "prediction_querier",
    srcs = [
        "prediction_querier.cc",
    ],
    hdrs = [
        "prediction_querier.h",
    ],
    deps = [
        "//modules/common/math",
        "//modules/common/math:path_matcher",
        "//modules/planning/common:obstacle",
        "//modules/planning/proto:lattice_structure_proto",
        "//modules/planning/proto:planning_proto",
    ],
)

cpplint()
