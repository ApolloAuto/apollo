load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "open_space_planner",
    srcs = [
        "open_space_planner.cc",
    ],
    hdrs = [
        "open_space_planner.h",
    ],
    deps = [
        "//modules/planning/open_space:warm_start_problem",
        "//modules/planning/open_space:distance_approach_problem",
        "//modules/planning/open_space:hybrid_a_star",
        "//external:gflags",
        "//cyber/common:log",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap",
        "//modules/planning/common:planning_common",
        "//modules/planning/planner",
        "//modules/planning/common:frame_open_space",
        "//modules/planning/common:frame",
        "//modules/planning/proto:planning_proto",
        "@eigen//:eigen",
    ],
)

cc_test(
    name = "open_space_planner_test",
    size = "small",
    srcs = [
        "open_space_planner_test.cc",
    ],
    deps = [
        "@gtest//:main",
        ":open_space_planner",
    ],
)

cpplint()
