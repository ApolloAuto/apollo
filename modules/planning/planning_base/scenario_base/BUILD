load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])
PLANNING_COPTS = ["-DMODULE_NAME=\\\"planning\\\""]

cc_library(
    name = "scenario",
    srcs = ["scenario.cc"],
    hdrs = ["scenario.h"],
    copts = PLANNING_COPTS,
    deps = [
        ":stage",
        ":process_result",
        "//modules/planning/planning_base/common:planning_common",
        "//modules/planning/planning_base/common:config_util",
        "//modules/planning/planning_base/common/util:util_lib",
        "//cyber/plugin_manager:plugin_manager",
        "//modules/planning/planning_base/scenario_base/proto:scenario_pipeline_cc_proto",
    ],
)

cc_library(
    name = "stage",
    srcs = [
        "stage.cc",
        "base_stage_creep.cc",
        "base_stage_cruise.cc",
    ],
    hdrs = [
        "stage.h",
        "base_stage_creep.h",
        "base_stage_cruise.h",
    ],
    copts = PLANNING_COPTS,
    deps = [
        ":process_result",
        "//modules/planning/planning_base/common/util:common_lib",
        "//modules/planning/planning_base/common:planning_common",
        "//modules/planning/planning_base/common:config_util",
        "//modules/planning/planning_base/common/util:util_lib",
        "//cyber/plugin_manager:plugin_manager",
        "//modules/planning/planning_base/scenario_base/proto:scenario_pipeline_cc_proto",
        "//modules/planning/planning_base/common:speed_profile_generator",
        "//modules/planning/planning_base/common:dependency_injector",
        "//modules/planning/planning_base/task_base:task",
        "//modules/planning/planning_base/scenario_base/proto:creep_stage_cc_proto",
    ],
)

cc_library(
    name = "scenario_manager",
    srcs = ["scenario_manager.cc"],
    hdrs = ["scenario_manager.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/planning_base/common:planning_common",
        "//modules/planning/planning_base/common:planning_context",
        "//modules/planning/planning_base/common/util:util_lib",
        "//cyber/plugin_manager:plugin_manager",
        ":scenario",
    ],
)

cc_library(
    name = "process_result",
    srcs = ["process_result.cc"],
    hdrs = ["process_result.h"],
    copts = PLANNING_COPTS,
    deps = [
        "//modules/common/status:status",
    ],
)

cpplint()
apollo_package()
