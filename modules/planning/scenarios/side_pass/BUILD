load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "side_pass",
    srcs = [
        "side_pass_scenario.cc",
        "stage_approach_obstacle.cc",
        "stage_backup.cc",
        "stage_detect_safety.cc",
        "stage_generate_path.cc",
        "stage_pass_obstacle.cc",
        "stage_stop_on_wait_point.cc",
    ],
    hdrs = [
        "side_pass_scenario.h",
        "stage_approach_obstacle.h",
        "stage_backup.h",
        "stage_detect_safety.h",
        "stage_generate_path.h",
        "stage_pass_obstacle.h",
        "stage_stop_on_wait_point.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//modules/common",
        "//modules/common/util:factory",
        "//modules/planning/common:obstacle_blocking_analyzer",
        "//modules/planning/common:planning_common",
        "//modules/planning/common:speed_profile_generator",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/reference_line",
        "//modules/planning/scenarios:scenario",
        "@eigen",
    ],
)

cc_test(
    name = "side_pass_scenario_test",
    size = "small",
    srcs = [
        "side_pass_scenario_test.cc",
    ],
    data = [
        "//modules/planning:planning_conf",
    ],
    deps = [
        ":side_pass",
        "@gtest//:main",
    ],
)

cpplint()
