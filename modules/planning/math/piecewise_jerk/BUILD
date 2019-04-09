load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "piecewise_jerk_problem",
    srcs = [
        "piecewise_jerk_problem.cc",
    ],
    hdrs = [
        "piecewise_jerk_problem.h",
    ],
    deps = [
        "//cyber/common:log",
        "//modules/planning/common:planning_gflags",
        "@osqp",
    ],
)

cc_test(
    name = "piecewise_jerk_problem_test",
    size = "small",
    srcs = [
        "piecewise_jerk_problem.cc",
    ],
    deps = [
        ":piecewise_jerk_problem",
        "//cyber/common:log",
        "@gtest//:main",
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
        ":piecewise_jerk_problem",
    ],
)

cc_library(
    name = "path_time_qp_problem",
    srcs = [
        "path_time_qp_problem.cc",
    ],
    hdrs = [
        "path_time_qp_problem.h",
    ],
    deps = [
        ":piecewise_jerk_problem",
    ],
)
cpplint()
