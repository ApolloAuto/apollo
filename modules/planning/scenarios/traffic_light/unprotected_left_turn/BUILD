load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "traffic_light_unprotected_left_turn_scenario",
    srcs = [
        "stage_creep.cc",
        "stage_intersection_cruise.cc",
        "traffic_light_unprotected_left_turn_scenario.cc",
    ],
    hdrs = [
        "stage_creep.h",
        "stage_intersection_cruise.h",
        "traffic_light_unprotected_left_turn_scenario.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//external:gflags",
        "//modules/common",
        "//modules/common/util",
        "//modules/common/util:factory",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/scenarios:scenario",
        "//modules/planning/scenarios:stage_intersection_cruise_impl",
        "//modules/planning/tasks/deciders:decider",
        "@eigen",
    ],
)

cc_test(
    name = "traffic_light_left_turn_unprotected_scenario_test",
    size = "small",
    srcs = ["traffic_light_unprotected_left_turn_scenario_test.cc"],
    data = ["//modules/planning:planning_conf"],
    deps = [
        ":traffic_light_unprotected_left_turn_scenario",
        "@gtest//:main",
    ],
)

cpplint()
