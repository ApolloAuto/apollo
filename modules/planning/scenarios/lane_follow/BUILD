load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_follow",
    hdrs = [
        "lane_follow_scenario.h",
    ],
    deps = [
        "//modules/common",
        "//modules/planning/common:planning_common",
        "//modules/planning/scenarios:scenario_lib",
    ],
)

cc_test(
    name = "lane_follow_scenario_test",
    size = "small",
    srcs = [
        "lane_follow_scenario_test.cc",
    ],
    deps = [
        ":lane_follow",
        "@gtest//:main",
    ],
)

cpplint()
