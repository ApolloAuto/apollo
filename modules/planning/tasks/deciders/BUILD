load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "decider",
    srcs = [
        "decider_creep.cc",
    ],
    hdrs = [
        "decider_creep.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":decider_base",
        ":open_space_fallback_decider",
        ":open_space_pre_stop_decider",
        ":open_space_roi_decider",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common:st_graph_data",
        "//modules/planning/common/util:common_lib",
        "//modules/planning/common/util:util_lib",
        "//modules/planning/math/piecewise_jerk:fem_1d_qp_problem",
        "//modules/planning/scenarios/util:scenario_util_lib",
        "//modules/planning/tasks/deciders/speed_bounds_decider",
        "//modules/planning/tasks/deciders/speed_bounds_decider:st_boundary_mapper",
    ],
)

cc_library(
    name = "open_space_pre_stop_decider",
    srcs = [
        "open_space_pre_stop_decider.cc",
    ],
    hdrs = [
        "open_space_pre_stop_decider.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":decider_base",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common/util:common_lib",
    ],
)

cc_library(
    name = "path_assessment_decider",
    srcs = [
        "path_assessment_decider.cc",
    ],
    hdrs = [
        "path_assessment_decider.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":decider_base",
        ":path_bounds_decider",
        ":path_decider_obstacle_utils",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
    ],
)

cc_library(
    name = "path_bounds_decider",
    srcs = [
        "path_bounds_decider.cc",
    ],
    hdrs = [
        "path_bounds_decider.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":decider_base",
        ":path_decider_obstacle_utils",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
    ],
)

cc_library(
    name = "path_decider_obstacle_utils",
    srcs = [
        "path_decider_obstacle_utils.cc",
    ],
    hdrs = [
        "path_decider_obstacle_utils.h",
    ],
    deps = [
        "//modules/planning/common:frame",
    ],
)

cc_library(
    name = "decider_base",
    srcs = [
        "decider.cc",
    ],
    hdrs = [
        "decider.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:frame",
        "//modules/planning/tasks:task",
    ],
)

cc_library(
    name = "open_space_fallback_decider",
    srcs = [
        "open_space_fallback_decider.cc",
    ],
    hdrs = [
        "open_space_fallback_decider.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":decider_base",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
    ],
)

cc_library(
    name = "open_space_roi_decider",
    srcs = [
        "open_space_roi_decider.cc",
    ],
    hdrs = [
        "open_space_roi_decider.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":decider_base",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
    ],
)

cc_test(
    name = "open_space_fallback_decider_test",
    size = "small",
    srcs = [
        "open_space_fallback_decider_test.cc",
    ],
    deps = [
        ":open_space_fallback_decider",
        "@gtest//:main",
    ],
)

cc_test(
    name = "open_space_roi_decider_test",
    size = "small",
    srcs = [
        "open_space_roi_decider_test.cc",
    ],
    deps = [
        ":open_space_roi_decider",
        "@gtest//:main",
    ],
)

cc_test(
    name = "path_bounds_decider_test",
    size = "small",
    srcs = [
        "path_bounds_decider_test.cc",
    ],
    deps = [
        "path_bounds_decider",
        "@gtest//:main",
    ],
)

cpplint()
