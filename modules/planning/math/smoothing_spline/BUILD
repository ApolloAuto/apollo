load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "affine_constraint",
    srcs = [
        "affine_constraint.cc",
    ],
    hdrs = [
        "affine_constraint.h",
    ],
    deps = [
        "//modules/planning/math:polynomial_xd",
        "@eigen",
    ],
)

cc_library(
    name = "spline_1d_seg",
    srcs = [
        "spline_1d_seg.cc",
    ],
    hdrs = [
        "spline_1d_seg.h",
    ],
    deps = [
        "//modules/planning/math:polynomial_xd",
        "@eigen",
    ],
)

cc_library(
    name = "spline_1d",
    srcs = [
        "spline_1d.cc",
    ],
    hdrs = [
        "spline_1d.h",
    ],
    deps = [
        ":affine_constraint",
        ":spline_1d_seg",
        "//modules/planning/math:polynomial_xd",
        "@eigen",
    ],
)

cc_library(
    name = "spline_1d_constraint",
    srcs = [
        "spline_1d_constraint.cc",
    ],
    hdrs = [
        "spline_1d_constraint.h",
    ],
    deps = [
        ":affine_constraint",
        ":spline_1d",
        "//modules/common:log",
        "@eigen",
    ],
)

cc_library(
    name = "spline_seg_kernel",
    srcs = [
        "spline_seg_kernel.cc",
    ],
    hdrs = [
        "spline_seg_kernel.h",
    ],
    deps = [
        "//modules/common:macro",
        "@eigen",
    ],
)

cc_library(
    name = "spline_1d_kernel",
    srcs = [
        "spline_1d_kernel.cc",
    ],
    hdrs = [
        "spline_1d_kernel.h",
    ],
    deps = [
        ":affine_constraint",
        ":spline_1d",
        ":spline_seg_kernel",
        "//modules/common:macro",
        "@eigen",
    ],
)

cc_library(
    name = "spline_1d_generator",
    srcs = [
        "spline_1d_generator.cc",
    ],
    hdrs = [
        "spline_1d_generator.h",
    ],
    deps = [
        ":spline_1d",
        ":spline_1d_constraint",
        ":spline_1d_kernel",
        "//modules/common/math/qp_solver",
        "//modules/common/math/qp_solver:active_set_qp_solver",
        "//modules/common/time",
        "//modules/planning/common:planning_gflags",
        "@eigen",
    ],
)

cc_library(
    name = "spline_2d_seg",
    srcs = [
        "spline_2d_seg.cc",
    ],
    hdrs = [
        "spline_2d_seg.h",
    ],
    deps = [
        "//modules/planning/math:polynomial_xd",
        "@eigen",
    ],
)

cc_library(
    name = "spline_2d",
    srcs = [
        "spline_2d.cc",
    ],
    hdrs = [
        "spline_2d.h",
    ],
    deps = [
        ":spline_2d_seg",
        "//modules/planning/math:polynomial_xd",
        "@eigen",
    ],
)

cc_library(
    name = "spline_2d_constraint",
    srcs = [
        "spline_2d_constraint.cc",
    ],
    hdrs = [
        "spline_2d_constraint.h",
    ],
    deps = [
        ":affine_constraint",
        ":spline_2d",
        "//modules/common/math:geometry",
        "@eigen",
    ],
)

cc_library(
    name = "spline_2d_kernel",
    srcs = [
        "spline_2d_kernel.cc",
    ],
    hdrs = [
        "spline_2d_kernel.h",
    ],
    deps = [
        ":spline_2d",
        ":spline_seg_kernel",
        "//modules/common/math:geometry",
        "@eigen",
    ],
)

cc_library(
    name = "spline_2d_solver",
    srcs = [
        "spline_2d_solver.cc",
    ],
    hdrs = [
        "spline_2d_solver.h",
    ],
    deps = [
        ":spline_2d",
        ":spline_2d_constraint",
        ":spline_2d_kernel",
        "//modules/common/math:geometry",
        "//modules/common/math/qp_solver",
        "//modules/common/math/qp_solver:active_set_qp_solver",
        "//modules/common/time",
        "//modules/planning/common:planning_gflags",
        "@eigen",
    ],
)

cc_library(
    name = "piecewise_linear_constraint",
    srcs = [
        "piecewise_linear_constraint.cc",
    ],
    hdrs = [
        "piecewise_linear_constraint.h",
    ],
    deps = [
        "//modules/common:log",
        "@eigen",
    ],
)

cc_library(
    name = "piecewise_linear_kernel",
    srcs = [
        "piecewise_linear_kernel.cc",
    ],
    hdrs = [
        "piecewise_linear_kernel.h",
    ],
    deps = [
        "//modules/common:log",
        "@eigen",
    ],
)

cc_library(
    name = "piecewise_linear_generator",
    srcs = [
        "piecewise_linear_generator.cc",
    ],
    hdrs = [
        "piecewise_linear_generator.h",
    ],
    deps = [
        ":piecewise_linear_constraint",
        ":piecewise_linear_kernel",
        "//modules/common/math/qp_solver",
        "//modules/common/math/qp_solver:active_set_qp_solver",
        "@eigen",
    ],
)

cc_test(
    name = "spline_1d_generator_test",
    size = "small",
    srcs = [
        "spline_1d_generator_test.cc",
    ],
    deps = [
        ":spline_1d_generator",
        "@gtest//:main",
    ],
)

cc_test(
    name = "spline_1d_kernel_test",
    size = "small",
    srcs = [
        "spline_1d_kernel_test.cc",
    ],
    deps = [
        ":spline_1d_kernel",
        "@gtest//:main",
    ],
)

cc_test(
    name = "spline_1d_constraint_test",
    size = "small",
    srcs = [
        "spline_1d_constraint_test.cc",
    ],
    deps = [
        ":spline_1d_constraint",
        "@gtest//:main",
    ],
)

cc_test(
    name = "piecewise_linear_kernel_test",
    size = "small",
    srcs = [
        "piecewise_linear_kernel_test.cc",
    ],
    deps = [
        ":piecewise_linear_kernel",
        "@gtest//:main",
    ],
)

cc_test(
    name = "piecewise_linear_constraint_test",
    size = "small",
    srcs = [
        "piecewise_linear_constraint_test.cc",
    ],
    deps = [
        ":piecewise_linear_constraint",
        "@gtest//:main",
    ],
)

cc_test(
    name = "spline_2d_solver_test",
    size = "small",
    srcs = [
        "spline_2d_solver_test.cc",
    ],
    deps = [
        ":spline_2d_solver",
        "//modules/planning/math:curve_math",
        "@gtest//:main",
    ],
)

cc_test(
    name = "spline_2d_kernel_test",
    size = "small",
    srcs = [
        "spline_2d_kernel_test.cc",
    ],
    deps = [
        ":spline_2d_kernel",
        "@gtest//:main",
    ],
)

cc_test(
    name = "spline_2d_constraint_test",
    size = "small",
    srcs = [
        "spline_2d_constraint_test.cc",
    ],
    deps = [
        ":spline_2d_constraint",
        "@gtest//:main",
    ],
)

cpplint()
