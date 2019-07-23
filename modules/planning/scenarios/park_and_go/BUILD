load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "park_and_go_scenario",
    srcs = [
        "park_and_go_scenario.cc",
        "stage_check.cc",
    ],
    hdrs = [
        "park_and_go_scenario.h",
        "stage_check.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//external:gflags",
        "//modules/common",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:planning_common",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/scenarios:scenario",
        "//modules/planning/scenarios/util:scenario_util_lib",
    ],
)

cc_test(
    name = "park_and_go_scenario_test",
    size = "small",
    srcs = [
        "park_and_go_scenario_test.cc",
    ],
    data = [
        "//modules/planning:planning_conf",
    ],
    deps = [
        ":park_and_go_scenario",
        "@gtest//:main",
    ],
)

cc_test(
    name = "stage_check_test",
    size = "small",
    srcs = [
        "stage_check_test.cc",
    ],
    data = [
        "//modules/planning:planning_conf",
    ],
    deps = [
        ":park_and_go_scenario",
        "@gtest//:main",
    ],
)

cpplint()
