load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "bare_intersection_unprotected_scenario",
    srcs = [
        "bare_intersection_unprotected_scenario.cc",
        "stage_approach.cc",
        "stage_intersection_cruise.cc",
    ],
    hdrs = [
        "bare_intersection_unprotected_scenario.h",
        "stage_approach.h",
        "stage_intersection_cruise.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//modules/common/util",
        "//modules/common/util:factory",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/scenarios:scenario",
        "//modules/planning/scenarios/common:stage_intersection_cruise_impl",
        "//modules/planning/tasks/deciders/creep_decider",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

cc_test(
    name = "bare_intersection_unprotected_scenario_test",
    size = "small",
    srcs = ["bare_intersection_unprotected_scenario_test.cc"],
    data = ["//modules/planning:planning_conf"],
    deps = [
        ":bare_intersection_unprotected_scenario",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "stage_approach_test",
    size = "small",
    srcs = ["stage_approach_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    deps = [
        ":bare_intersection_unprotected_scenario",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
