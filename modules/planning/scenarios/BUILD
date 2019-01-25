load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "scenario",
    srcs = [
        "scenario.cc",
    ],
    hdrs = [
        "scenario.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":stage",
        "//modules/common",
        "//modules/planning/common:planning_common",
        "//modules/planning/tasks:task",
    ],
)

cc_library(
    name = "stage",
    srcs = [
        "stage.cc",
    ],
    hdrs = [
        "stage.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common",
        "//modules/planning/common:planning_common",
        "//modules/planning/tasks:task",
        "//modules/planning/tasks:task_factory",
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
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common",
        "//modules/planning/common:planning_common",
        "//modules/planning/common:planning_context",
        "//modules/planning/scenarios/lane_follow",
        "//modules/planning/scenarios/side_pass",
        "//modules/planning/scenarios/stop_sign/unprotected:stop_sign_unprotected_scenario",
        "//modules/planning/scenarios/traffic_light/protected:traffic_light_protected_scenario",
        "//modules/planning/scenarios/traffic_light/unprotected_right_turn:traffic_light_unprotected_right_turn_scenario",
    ],
)

cpplint()
