load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "trajectory_point_collector",
    srcs = [
        "trajectory_point_collector.cc",
    ],
    hdrs = [
        "trajectory_point_collector.h",
    ],
    deps = [
        "//modules/dreamview/proto:simulation_world_proto",
        "//modules/planning/proto:planning_proto",
    ],
)

cc_test(
    name = "trajectory_point_collector_test",
    size = "small",
    srcs = [
        "trajectory_point_collector_test.cc",
    ],
    deps = [
        ":trajectory_point_collector",
        "@gtest//:main",
    ],
)

cpplint()
