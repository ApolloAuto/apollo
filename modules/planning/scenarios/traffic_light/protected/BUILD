load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "traffic_light_protected_scenario",
    srcs = [
        "stage_intersection_cruise.cc",
        "stage_stop.cc",
        "traffic_light_protected_scenario.cc",
    ],
    hdrs = [
        "stage_intersection_cruise.h",
        "stage_stop.h",
        "traffic_light_protected_scenario.h",
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
    name = "traffic_light_protected_scenario_test",
    size = "small",
    srcs = ["traffic_light_protected_scenario_test.cc"],
    data = ["//modules/planning:planning_conf"],
    deps = [
        ":traffic_light_protected_scenario",
        "@gtest//:main",
    ],
)

cpplint()
