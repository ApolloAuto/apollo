load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "park_and_go_scenario",
    srcs = [
        "park_and_go_scenario.cc",
        "stage_adjust.cc",
        "stage_check.cc",
        "stage_cruise.cc",
        "stage_pre_cruise.cc",
    ],
    hdrs = [
        "park_and_go_scenario.h",
        "stage_adjust.h",
        "stage_check.h",
        "stage_cruise.h",
        "stage_pre_cruise.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:planning_common",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/scenarios:scenario",
        "//modules/planning/scenarios/util:scenario_util_lib",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_test(
    name = "park_and_go_scenario_test",
    size = "small",
    srcs = ["park_and_go_scenario_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    deps = [
        ":park_and_go_scenario",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "park_and_go_stages_test",
    size = "small",
    srcs = [
        "stage_adjust_test.cc",
        "stage_check_test.cc",
        "stage_cruise_test.cc",
        "stage_pre_cruise_test.cc",
    ],
    data = [
        "//modules/planning:planning_conf",
    ],
    deps = [
        ":park_and_go_scenario",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
