load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "fem_1d_qp_problem",
    srcs = [
        "fem_1d_qp_problem.cc",
    ],
    hdrs = [
        "fem_1d_qp_problem.h",
    ],
    deps = [
        "//cyber/common:log",
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
        "//cyber/common:log",
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
        "//cyber/common:log",
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
        "//cyber/common:log",
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
        "//cyber/common:log",
        "@gtest//:main",
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
        "//cyber/common:log",
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
        "//cyber/common:log",
        "//modules/planning/common:planning_gflags",
        "@gtest//:main",
    ],
)

cpplint()
