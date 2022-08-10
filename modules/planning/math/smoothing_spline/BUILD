load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "affine_constraint",
    srcs = ["affine_constraint.cc"],
    hdrs = ["affine_constraint.h"],
    deps = [
        "//modules/planning/math:polynomial_xd",
        "@eigen",
    ],
)

cc_library(
    name = "spline_1d_seg",
    srcs = ["spline_1d_seg.cc"],
    hdrs = ["spline_1d_seg.h"],
    deps = [
        "//modules/planning/math:polynomial_xd",
        "@eigen",
    ],
)

cc_library(
    name = "spline_1d",
    srcs = ["spline_1d.cc"],
    hdrs = ["spline_1d.h"],
    deps = [
        ":affine_constraint",
        ":spline_1d_seg",
        "//modules/planning/math:polynomial_xd",
        "@eigen",
    ],
)

cc_library(
    name = "spline_1d_constraint",
    srcs = ["spline_1d_constraint.cc"],
    hdrs = ["spline_1d_constraint.h"],
    deps = [
        ":affine_constraint",
        ":spline_1d",
        "//cyber",
        "@eigen",
    ],
)

cc_library(
    name = "spline_seg_kernel",
    srcs = ["spline_seg_kernel.cc"],
    hdrs = ["spline_seg_kernel.h"],
    copts = [
        "-DMODULE_NAME=\\\"planning\\\"",
    ],
    deps = [
        "//cyber",
        "@eigen",
    ],
)

cc_library(
    name = "spline_1d_kernel",
    srcs = ["spline_1d_kernel.cc"],
    hdrs = ["spline_1d_kernel.h"],
    deps = [
        ":affine_constraint",
        ":spline_1d",
        ":spline_seg_kernel",
        "@eigen",
    ],
)

cc_library(
    name = "spline_1d_solver",
    srcs = ["spline_1d_solver.cc"],
    hdrs = ["spline_1d_solver.h"],
    deps = [
        ":spline_1d",
        ":spline_1d_constraint",
        ":spline_1d_kernel",
        # "//modules/common/math",
        "//modules/common/math/qp_solver",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/proto/math:qp_problem_cc_proto",
        "@eigen",
    ],
)

cc_library(
    name = "osqp_spline_1d_solver",
    srcs = ["osqp_spline_1d_solver.cc"],
    hdrs = ["osqp_spline_1d_solver.h"],
    deps = [
        ":spline_1d_solver",
        "//modules/common/math",
        "//modules/common/math/qp_solver",
        "//modules/planning/common:planning_gflags",
        "@osqp",
    ],
)

cc_test(
    name = "osqp_spline_1d_solver_test",
    size = "small",
    srcs = ["osqp_spline_1d_solver_test.cc"],
    deps = [
        ":osqp_spline_1d_solver",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "spline_2d_seg",
    srcs = ["spline_2d_seg.cc"],
    hdrs = ["spline_2d_seg.h"],
    deps = [
        "//modules/planning/math:polynomial_xd",
        "@eigen",
    ],
)

cc_library(
    name = "spline_2d",
    srcs = ["spline_2d.cc"],
    hdrs = ["spline_2d.h"],
    deps = [
        ":spline_2d_seg",
        "//modules/planning/math:polynomial_xd",
        "@eigen",
    ],
)

cc_library(
    name = "spline_2d_constraint",
    srcs = ["spline_2d_constraint.cc"],
    hdrs = ["spline_2d_constraint.h"],
    deps = [
        ":affine_constraint",
        ":spline_2d",
        "//modules/common/math",
        "@eigen",
    ],
)

cc_library(
    name = "spline_2d_kernel",
    srcs = ["spline_2d_kernel.cc"],
    hdrs = ["spline_2d_kernel.h"],
    deps = [
        ":spline_2d",
        ":spline_seg_kernel",
        "//modules/common/math",
        "@eigen",
    ],
)

cc_library(
    name = "spline_2d_solver",
    hdrs = [
        "osqp_spline_2d_solver.h",
        "spline_2d_solver.h",
    ],
    deps = [
        ":spline_2d",
        ":spline_2d_constraint",
        ":spline_2d_kernel",
        # "//modules/common/math",
        "//modules/common/math/qp_solver",
        "//modules/planning/common:planning_gflags",
        "@eigen",
        "@osqp",
    ],
)

cc_library(
    name = "osqp_spline_2d_solver",
    srcs = ["osqp_spline_2d_solver.cc"],
    hdrs = ["osqp_spline_2d_solver.h"],
    deps = [
        ":spline_2d_solver",
        ":spline_2d",
        "//modules/common/math",
        "//modules/planning/common:planning_gflags",
        "@com_google_googletest//:gtest",
        "@osqp",
    ],
)

cc_test(
    name = "osqp_spline_2d_solver_test",
    size = "small",
    srcs = ["osqp_spline_2d_solver_test.cc"],
    deps = [
        ":osqp_spline_2d_solver",
        "//modules/planning/math:curve_math",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "spline_1d_kernel_test",
    size = "small",
    srcs = ["spline_1d_kernel_test.cc"],
    linkopts = ["-lm"],
    deps = [
        ":spline_1d_kernel",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "spline_1d_constraint_test",
    size = "small",
    srcs = ["spline_1d_constraint_test.cc"],
    deps = [
        ":spline_1d_constraint",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "spline_2d_kernel_test",
    size = "small",
    srcs = ["spline_2d_kernel_test.cc"],
    deps = [
        ":spline_2d_kernel",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "spline_2d_constraint_test",
    size = "small",
    srcs = ["spline_2d_constraint_test.cc"],
    deps = [
        ":spline_2d_constraint",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
