load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "scenario_lib",
    srcs = [
        "scenario.cc",
    ],
    hdrs = [
        "scenario.h",
    ],
    deps = [
        "//modules/common",
        "//modules/planning/common:planning_common",
        "//modules/planning/scenarios/lane_follow",
    ],
)

cpplint()
