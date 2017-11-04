load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rtk_planner",
    srcs = [
        "rtk_replay_planner.cc",
    ],
    hdrs = [
        "rtk_replay_planner.h",
    ],
    deps = [
        "//external:gflags",
        "//modules/common:log",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/util",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:planning_common",
        "//modules/planning/math/curve1d:quartic_polynomial_curve1d",
        "//modules/planning/planner",
        "//modules/planning/proto:planning_proto",
        "@eigen//:eigen",
    ],
)

cc_test(
    name = "rtk_replay_planner_test",
    size = "small",
    srcs = [
        "rtk_replay_planner_test.cc",
    ],
    data = ["//modules/planning:planning_testdata"],
    deps = [
        ":rtk_planner",
        "//modules/common:log",
        "//modules/common/time",
        "//modules/common/util",
        "@gtest//:main",
    ],
)

cpplint()
