load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "navi_planner",
    srcs = ["navi_planner.cc"],
    hdrs = ["navi_planner.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/status",
        "//modules/common/util",
        "//modules/common/util:util_tool",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap",
        "//modules/planning/planning_base/common:planning_common",
        "//modules/planning/planning_base/proto:navi_task_config_cc_proto",
        "//modules/planning/planning_base/constraint_checker",
        "//modules/planning/planning_base/math/curve1d:quartic_polynomial_curve1d",
        "//modules/planning/planning_base/navi/decider:navi_obstacle_decider",
        "//modules/planning/planning_base/navi/decider:navi_path_decider",
        "//modules/planning/planning_base/navi/decider:navi_speed_decider",
        "//modules/planning/planning_base/planner",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/planning/planning_base/reference_line",
        "//modules/planning/planning_base/reference_line:qp_spline_reference_line_smoother",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_test(
    name = "navi_planner_test",
    size = "small",
    srcs = ["navi_planner_test.cc"],
    data = ["//modules/planning/planning_base:planning_testdata"],
    linkopts = ["-lgomp"],
    deps = [
        ":navi_planner",
        "//cyber",
        "//modules/common/util",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_package()
cpplint()
