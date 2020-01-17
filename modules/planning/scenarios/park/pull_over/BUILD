load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pull_over_scenario",
    srcs = [
        "pull_over_scenario.cc",
        "stage_approach.cc",
        "stage_retry_approach_parking.cc",
        "stage_retry_parking.cc",
    ],
    hdrs = [
        "pull_over_scenario.h",
        "stage_approach.h",
        "stage_retry_approach_parking.h",
        "stage_retry_parking.h",
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
        "@eigen",
    ],
)

cc_test(
    name = "pull_over_scenario_test",
    size = "small",
    srcs = ["pull_over_scenario_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    deps = [
        ":pull_over_scenario",
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
        ":pull_over_scenario",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "stage_retry_approach_parking_test",
    size = "small",
    srcs = ["stage_retry_approach_parking_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    deps = [
        ":pull_over_scenario",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "stage_retry_parking_test",
    size = "small",
    srcs = ["stage_retry_parking.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    deps = [
        ":pull_over_scenario",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
