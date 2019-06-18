load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "public_road_planner",
    srcs = [
        "public_road_planner.cc",
    ],
    hdrs = [
        "public_road_planner.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//external:gflags",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap",
        "//modules/planning/common:planning_common",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/constraint_checker",
        "//modules/planning/math/curve1d:quartic_polynomial_curve1d",
        "//modules/planning/planner",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/reference_line",
        "//modules/planning/reference_line:qp_spline_reference_line_smoother",
        "//modules/planning/tasks:task",
        "//modules/planning/tasks/deciders/path_decider",
        "//modules/planning/tasks/deciders/speed_decider",
        "//modules/planning/tasks/optimizers:path_optimizer",
        "//modules/planning/tasks/optimizers:speed_optimizer",
        "//modules/planning/tasks/optimizers/path_time_heuristic:path_time_heuristic_optimizer",
        "@eigen",
    ],
)

cc_test(
    name = "public_road_planner_test",
    size = "small",
    srcs = [
        "public_road_planner_test.cc",
    ],
    deps = [
        ":public_road_planner",
        "@gtest//:main",
    ],
)

cpplint()
