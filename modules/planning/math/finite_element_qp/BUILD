load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "finite_element_qp",
    deps = [
        ":lateral_qp_optimizer",
        ":osqp_lateral_jerk_qp_optimizer",
        ":osqp_lateral_linear_qp_optimizer",
        ":osqp_lateral_qp_optimizer",
    ],
)

cc_library(
    name = "lateral_qp_optimizer",
    srcs = [
        "lateral_qp_optimizer.cc",
    ],
    hdrs = [
        "lateral_qp_optimizer.h",
    ],
    deps = [
        "//cybertron",
        "//modules/planning/lattice/trajectory1d:piecewise_jerk_trajectory1d",
    ],
)

cc_library(
    name = "osqp_lateral_qp_optimizer",
    srcs = [
        "osqp_lateral_qp_optimizer.cc",
    ],
    hdrs = [
        "osqp_lateral_qp_optimizer.h",
    ],
    deps = [
        ":lateral_qp_optimizer",
        "@eigen",
        "@osqp",
    ],
)

cc_library(
    name = "osqp_lateral_linear_qp_optimizer",
    srcs = [
        "osqp_lateral_linear_qp_optimizer.cc",
    ],
    hdrs = [
        "osqp_lateral_linear_qp_optimizer.h",
    ],
    deps = [
        ":lateral_qp_optimizer",
        "@eigen",
        "@osqp",
    ],
)

cc_library(
    name = "osqp_lateral_jerk_qp_optimizer",
    srcs = [
        "osqp_lateral_jerk_qp_optimizer.cc",
    ],
    hdrs = [
        "osqp_lateral_jerk_qp_optimizer.h",
    ],
    deps = [
        ":lateral_qp_optimizer",
        "@eigen",
        "@osqp",
    ],
)

cc_library(
    name = "fem_1d_qp_problem",
    srcs = [
        "fem_1d_qp_problem.cc",
    ],
    hdrs = [
        "fem_1d_qp_problem.h",
    ],
    deps = [
        "//cybertron",
        "//modules/planning/common:planning_gflags",
        "@osqp",
    ],
)

cc_library(
    name = "fem_1d_expanded_qp_problem",
    srcs = [
        "fem_1d_expanded_qp_problem.cc",
    ],
    hdrs = [
        "fem_1d_expanded_qp_problem.h",
    ],
    deps = [
        ":fem_1d_qp_problem",
        "//cybertron",
        "//modules/common/math:matrix_operations",
        "@eigen",
    ],
)

cc_test(
    name = "fem_1d_expanded_qp_problem_test",
    size = "small",
    srcs = [
        "fem_1d_expanded_qp_problem_test.cc",
    ],
    deps = [
        ":fem_1d_expanded_qp_problem",
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_library(
    name = "fem_1d_linear_qp_problem",
    srcs = [
        "fem_1d_linear_qp_problem.cc",
    ],
    hdrs = [
        "fem_1d_linear_qp_problem.h",
    ],
    deps = [
        ":fem_1d_qp_problem",
        "//cybertron",
        "//modules/common/math:matrix_operations",
        "@eigen",
    ],
)

cc_test(
    name = "fem_1d_linear_qp_problem_test",
    size = "small",
    srcs = [
        "fem_1d_linear_qp_problem_test.cc",
    ],
    deps = [
        ":fem_1d_linear_qp_problem",
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_library(
    name = "fem_1d_jerk_qp_problem",
    srcs = [
        "fem_1d_jerk_qp_problem.cc",
    ],
    hdrs = [
        "fem_1d_jerk_qp_problem.h",
    ],
    deps = [
        ":fem_1d_qp_problem",
        "//cybertron",
        "//modules/common/math:matrix_operations",
        "@eigen",
    ],
)

cc_library(
    name = "fem_1d_expanded_jerk_qp_problem",
    srcs = [
        "fem_1d_expanded_jerk_qp_problem.cc",
    ],
    hdrs = [
        "fem_1d_expanded_jerk_qp_problem.h",
    ],
    deps = [
        ":fem_1d_qp_problem",
        "//cybertron",
        "//modules/common/math:matrix_operations",
        "@eigen",
    ],
)

cc_test(
    name = "fem_1d_expanded_jerk_qp_problem_test",
    size = "small",
    srcs = [
        "fem_1d_expanded_jerk_qp_problem_test.cc",
    ],
    deps = [
        ":fem_1d_expanded_jerk_qp_problem",
        "//cybertron",
        "//modules/planning/common:planning_gflags",
        "@gtest//:main",
    ],
)

cpplint()
