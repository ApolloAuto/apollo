load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_follow",
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":lane_follow_scenario",
        ":lane_follow_stage",
    ],
)

cc_library(
    name = "lane_follow_stage",
    srcs = ["lane_follow_stage.cc"],
    hdrs = ["lane_follow_stage.h"],
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
        "//modules/planning/common:speed_profile_generator",
        "//modules/planning/constraint_checker",
        "//modules/planning/math/curve1d:quartic_polynomial_curve1d",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/reference_line",
        "//modules/planning/reference_line:qp_spline_reference_line_smoother",
        "//modules/planning/scenarios:scenario",
        "//modules/planning/tasks/deciders/lane_change_decider",
        "//modules/planning/tasks/deciders/path_decider",
        "//modules/planning/tasks/deciders/speed_decider",
        "//modules/planning/tasks/optimizers:path_optimizer",
        "//modules/planning/tasks/optimizers:speed_optimizer",
        "//modules/planning/tasks/optimizers/path_time_heuristic:path_time_heuristic_optimizer",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

cc_library(
    name = "lane_follow_scenario",
    srcs = ["lane_follow_scenario.cc"],
    hdrs = ["lane_follow_scenario.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":lane_follow_stage",
        "//cyber/common:log",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/common/util:factory",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/map/hdmap",
        "//modules/planning/common:planning_common",
        "//modules/planning/common:speed_profile_generator",
        "//modules/planning/constraint_checker",
        "//modules/planning/math/curve1d:quartic_polynomial_curve1d",
        "//modules/planning/proto:planning_proto",
        "//modules/planning/reference_line",
        "//modules/planning/reference_line:qp_spline_reference_line_smoother",
        "//modules/planning/scenarios:scenario",
        "//modules/planning/tasks/deciders/path_decider",
        "//modules/planning/tasks/deciders/speed_decider",
        "//modules/planning/tasks/optimizers:path_optimizer",
        "//modules/planning/tasks/optimizers:speed_optimizer",
        "//modules/planning/tasks/optimizers/path_time_heuristic:path_time_heuristic_optimizer",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

cc_test(
    name = "lane_follow_scenario_test",
    size = "small",
    srcs = ["lane_follow_scenario_test.cc"],
    data = [
        "//modules/planning:planning_conf",
    ],
    deps = [
        ":lane_follow",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
