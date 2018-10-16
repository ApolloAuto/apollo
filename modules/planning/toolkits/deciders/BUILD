load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "decider",
    srcs = [
        "decider.cc",
        "decider_creep.cc",
        "decider_stop_sign.cc",
        "side_pass_path_decider.cc",
        "side_pass_safety.cc",
    ],
    hdrs = [
        "decider.h",
        "decider_creep.h",
        "decider_stop_sign.h",
        "side_pass_path_decider.h",
        "side_pass_safety.h",
    ],
    deps = [
        "//modules/common/status",
        "//modules/planning/common:reference_line_info",
        "//modules/planning/math/finite_element_qp:fem_1d_expanded_jerk_qp_problem",
        "//modules/planning/toolkits:task",
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
        "@gtest//:main",
        ":decider",
    ],
)

cpplint()
