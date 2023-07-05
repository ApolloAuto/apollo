load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "discretized_trajectory",
    srcs = ["discretized_trajectory.cc"],
    hdrs = ["discretized_trajectory.h"],
    deps = [
        "//modules/common/math",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/vehicle_state/proto:vehicle_state_cc_proto",
        "//modules/planning/common:planning_context",
        "@eigen",
    ],
)

cc_test(
    name = "discretized_trajectory_test",
    size = "small",
    srcs = ["discretized_trajectory_test.cc"],
    data = [
        "//modules/planning:planning_testdata",
    ],
    deps = [
        ":discretized_trajectory",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "publishable_trajectory",
    srcs = ["publishable_trajectory.cc"],
    hdrs = ["publishable_trajectory.h"],
    deps = [
        ":discretized_trajectory",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
    ],
)

cc_test(
    name = "publishable_trajectory_test",
    size = "small",
    srcs = ["publishable_trajectory_test.cc"],
    data = [
        "//modules/planning:planning_testdata",
    ],
    deps = [
        ":publishable_trajectory",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
