load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "scenario_lib",
    srcs = [
        "scenario.cc",
    ],
    hdrs = [
        "scenario.h",
    ],
    deps = [
        "//modules/common",
        "//modules/planning/common:planning_common",
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
    ],
)

cc_test(
    name = "scenario_manager_test",
    size = "small",
    srcs = [
        "scenario_manager_test.cc",
    ],
    deps = [
        ":scenario_manager",
        "@gtest//:main",
    ],
)

cpplint()
