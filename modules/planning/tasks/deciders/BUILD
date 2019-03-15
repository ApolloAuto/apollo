load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "decider",
    srcs = [
        "decider_creep.cc",
        "side_pass_path_decider.cc",
        "side_pass_safety.cc",
    ],
    hdrs = [
        "decider_creep.h",
        "side_pass_path_decider.h",
        "side_pass_safety.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":decider_base",
        ":decider_rule_based_stop",
        ":open_space_fallback_decider",
        ":open_space_roi_decider",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/common:st_graph_data",
        "//modules/planning/math/finite_element_qp:fem_1d_qp_problem",
        "//modules/planning/scenarios/util:util_lib",
        "//modules/planning/tasks/deciders/speed_bounds_decider",
        "//modules/planning/tasks/deciders/speed_bounds_decider:st_boundary_mapper",
    ],
)

cc_library(
    name = "decider_rule_based_stop",
    srcs = [
        "decider_rule_based_stop.cc",
    ],
    hdrs = [
        "decider_rule_based_stop.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":decider_base",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/scenarios/util:util_lib",
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
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
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
    name = "side_pass_decider_test",
    size = "small",
    srcs = [
        "side_pass_path_decider_test.cc",
        "side_pass_safety_test.cc",
    ],
    deps = [
        ":decider",
        "@gtest//:main",
    ],
)

cpplint()
