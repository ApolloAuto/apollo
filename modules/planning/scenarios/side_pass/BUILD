load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "side_pass",
    srcs = [
        "side_pass_scenario.cc",
        "side_pass_stage.cc",
        "side_pass_stop_on_wait_point.cc",
    ],
    hdrs = [
        "side_pass_scenario.h",
        "side_pass_stage.h",
        "side_pass_stop_on_wait_point.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//modules/common",
        "//modules/common/util:factory",
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
