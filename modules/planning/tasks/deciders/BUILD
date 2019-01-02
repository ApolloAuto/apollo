load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "decider",
    srcs = [
        "decider.cc",
        "decider_creep.cc",
        "decider_rule_based_stop.cc",
        "side_pass_path_decider.cc",
        "side_pass_safety.cc",
    ],
    hdrs = [
        "decider.h",
        "decider_creep.h",
        "decider_rule_based_stop.h",
        "side_pass_path_decider.h",
        "side_pass_safety.h",
    ],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/common/status",
        "//modules/perception/proto:perception_proto",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_context",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/math/finite_element_qp:fem_1d_expanded_jerk_qp_problem",
        "//modules/planning/tasks:task",
    ],
)

cc_test(
    name = "decider_test",
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
