load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stop_sign_unprotected_scenario",
    srcs = [
        "stage_creep.cc",
        "stage_intersection_cruise.cc",
        "stage_pre_stop.cc",
        "stage_stop.cc",
        "stop_sign_unprotected_scenario.cc",
    ],
    hdrs = [
        "stage_creep.h",
        "stage_intersection_cruise.h",
        "stage_pre_stop.h",
        "stage_stop.h",
        "stop_sign_unprotected_scenario.h",
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
    name = "stop_sign_unprotected_scenario_test",
    size = "small",
    srcs = ["stop_sign_unprotected_scenario_test.cc"],
    data = ["//modules/planning:planning_conf"],
    deps = [
        ":stop_sign_unprotected_scenario",
        "@gtest//:main",
    ],
)

cpplint()
