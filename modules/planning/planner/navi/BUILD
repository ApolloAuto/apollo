load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "navi_planner",
    srcs = ["navi_planner.cc"],
    hdrs = ["navi_planner.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber/common:log",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap",
        "//modules/planning/common:planning_common",
        "//modules/planning/constraint_checker",
        "//modules/planning/math/curve1d:quartic_polynomial_curve1d",
        "//modules/planning/navi/decider:navi_obstacle_decider",
        "//modules/planning/navi/decider:navi_path_decider",
        "//modules/planning/navi/decider:navi_speed_decider",
        "//modules/planning/planner",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/reference_line",
        "//modules/planning/reference_line:qp_spline_reference_line_smoother",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_test(
    name = "navi_planner_test",
    size = "small",
    srcs = ["navi_planner_test.cc"],
    data = ["//modules/planning:planning_testdata"],
    deps = [
        ":navi_planner",
        "//cyber/common:log",
        "//modules/common/time",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
