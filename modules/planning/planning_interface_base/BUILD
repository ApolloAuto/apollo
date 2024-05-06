load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "apollo_planning_planning_interface_base",
    srcs = [
        "scenario_base/base_stage_creep.cc",
        "scenario_base/base_stage_cruise.cc",
        "scenario_base/process_result.cc",
        "scenario_base/scenario.cc",
        "scenario_base/stage.cc",
        "scenario_base/traffic_light_base/base_stage_traffic_light_creep.cc",
        "scenario_base/traffic_light_base/base_stage_traffic_light_cruise.cc",
        "task_base/common/decider.cc",
        "task_base/common/lane_change_util/lane_change_util.cc",
        "task_base/common/path_generation.cc",
        "task_base/common/path_util/path_assessment_decider_util.cc",
        "task_base/common/path_util/path_bounds_decider_util.cc",
        "task_base/common/path_util/path_optimizer_util.cc",
        "task_base/common/speed_optimizer.cc",
        "task_base/common/trajectory_optimizer.cc",
        "task_base/task.cc",
        "task_base/trajectory_fallback_task.cc",
        "task_base/utils/st_gap_estimator.cc",
        "traffic_rules_base/traffic_decider.cc",
        "traffic_rules_base/traffic_rule.cc",
    ],
    hdrs = [
        "scenario_base/base_stage_creep.h",
        "scenario_base/base_stage_cruise.h",
        "scenario_base/process_result.h",
        "scenario_base/scenario.h",
        "scenario_base/stage.h",
        "scenario_base/traffic_light_base/base_stage_traffic_light_creep.h",
        "scenario_base/traffic_light_base/base_stage_traffic_light_cruise.h",
        "task_base/common/decider.h",
        "task_base/common/lane_change_util/lane_change_util.h",
        "task_base/common/path_generation.h",
        "task_base/common/path_util/path_assessment_decider_util.h",
        "task_base/common/path_util/path_bounds_decider_util.h",
        "task_base/common/path_util/path_optimizer_util.h",
        "task_base/common/speed_optimizer.h",
        "task_base/common/trajectory_optimizer.h",
        "task_base/task.h",
        "task_base/trajectory_fallback_task.h",
        "task_base/utils/st_gap_estimator.h",
        "traffic_rules_base/traffic_decider.h",
        "traffic_rules_base/traffic_rule.h",
        "planner_base/planner.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/planning/planning_base:apollo_planning_planning_base",
        "//modules/planning/planning_interface_base/scenario_base/proto:scenario_pipeline_proto",
        "//modules/planning/planning_interface_base/scenario_base/proto:creep_stage_proto",
        "//modules/planning/planning_interface_base/traffic_rules_base/proto:traffic_rules_proto",
    ],
)

apollo_package()

cpplint()
