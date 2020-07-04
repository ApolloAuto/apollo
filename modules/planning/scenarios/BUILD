load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "scenario",
    srcs = ["scenario.cc"],
    hdrs = ["scenario.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":stage",
        "//modules/planning/common:planning_common",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/tasks:task",
    ],
)

cc_library(
    name = "stage",
    srcs = ["stage.cc"],
    hdrs = ["stage.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:planning_common",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/tasks:task",
        "//modules/planning/tasks:task_factory",
    ],
)

cc_library(
    name = "scenario_manager",
    srcs = ["scenario_manager.cc"],
    hdrs = ["scenario_manager.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/configs:vehicle_config_helper",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:planning_common",
        "//modules/planning/common:planning_context",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/scenarios/bare_intersection/unprotected:bare_intersection_unprotected_scenario",
        "//modules/planning/scenarios/emergency/emergency_pull_over:emergency_pull_over_scenario",
        "//modules/planning/scenarios/emergency/emergency_stop:emergency_stop_scenario",
        "//modules/planning/scenarios/lane_follow",
        "//modules/planning/scenarios/learning_model",
        "//modules/planning/scenarios/park/pull_over:pull_over_scenario",
        "//modules/planning/scenarios/park/valet_parking",
        "//modules/planning/scenarios/park_and_go:park_and_go_scenario",
        "//modules/planning/scenarios/stop_sign/unprotected:stop_sign_unprotected_scenario",
        "//modules/planning/scenarios/traffic_light/protected:traffic_light_protected_scenario",
        "//modules/planning/scenarios/traffic_light/unprotected_left_turn:traffic_light_unprotected_left_turn_scenario",
        "//modules/planning/scenarios/traffic_light/unprotected_right_turn:traffic_light_unprotected_right_turn_scenario",
        "//modules/planning/scenarios/yield_sign:yield_sign_scenario",
    ],
)

cpplint()
