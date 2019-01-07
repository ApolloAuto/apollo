load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "traffic_light_right_turn_unprotected_scenario",
    srcs = [
        "traffic_light_right_turn_unprotected_creep.cc",
        "traffic_light_right_turn_unprotected_intersection_cruise.cc",
        "traffic_light_right_turn_unprotected_scenario.cc",
        "traffic_light_right_turn_unprotected_stop.cc",
    ],
    hdrs = [
        "traffic_light_right_turn_unprotected_creep.h",
        "traffic_light_right_turn_unprotected_intersection_cruise.h",
        "traffic_light_right_turn_unprotected_scenario.h",
        "traffic_light_right_turn_unprotected_stop.h",
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
        "//modules/planning/tasks/deciders:decider",
        "@eigen",
    ],
)

cc_test(
    name = "traffic_light_right_turn_unprotected_scenario_test",
    size = "small",
    srcs = ["traffic_light_right_turn_unprotected_scenario_test.cc"],
    data = ["//modules/planning:planning_conf"],
    deps = [
        ":traffic_light_right_turn_unprotected_scenario",
        "@gtest//:main",
    ],
)

cpplint()
