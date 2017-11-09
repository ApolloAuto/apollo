load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "behavior_decider",
    hdrs = [
        "behavior_decider.h",
        "scenario.h",
        "scenario_manager.h",
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
        "//modules/planning/lattice:lattice_params",
        "//modules/planning/reference_line",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/proto:common_proto",
        "//modules/planning/lattice:lattice_util",
        "//modules/planning/lattice:reference_line_matcher",
        "//external:gflags",
    ],
)

cc_library(
    name = "scenario_manager",
    hdrs = [
        "scenario.h",
        "adc_master_scenario.h",
        "scenario_manager.h",
    ],
    srcs = [
        "scenario_manager.cc",
    ],
    deps = [
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
        "//modules/planning/proto:lattice_sampling_config_proto",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/proto:common_proto",
        "//modules/planning/common:frame",
    ],
)


cpplint()
