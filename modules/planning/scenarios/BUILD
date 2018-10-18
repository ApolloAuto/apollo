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
    deps = [
        ":stage",
        "//modules/common",
        "//modules/planning/common:planning_common",
        "//modules/planning/toolkits:task",
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
    deps = [
        "//modules/common",
        "//modules/planning/common:planning_common",
        "//modules/planning/toolkits:task",
        "//modules/planning/toolkits:task_factory",
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
        "//modules/common",
        "//modules/planning/common:planning_common",
        "//modules/planning/scenarios/lane_follow",
        "//modules/planning/scenarios/side_pass",
        "//modules/planning/scenarios/stop_sign_unprotected",
    ],
)

cpplint()
