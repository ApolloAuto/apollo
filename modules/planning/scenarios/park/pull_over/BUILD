load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pull_over_scenario",
    srcs = [
        "pull_over_scenario.cc",
        "stage_approach.cc",
    ],
    hdrs = [
        "pull_over_scenario.h",
        "stage_approach.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//external:gflags",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:planning_common",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/scenarios:scenario",
        "@eigen",
    ],
)

cc_test(
    name = "pull_over_scenario_test",
    size = "small",
    srcs = [
        "pull_over_scenario_test.cc",
    ],
    data = [
        "//modules/planning:planning_conf",
    ],
    deps = [
        ":pull_over_scenario",
        "@gtest//:main",
    ],
)

cpplint()
