load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "math",
    deps = [
        ":aabox2d",
        ":aaboxkdtree2d",
        ":angle",
        ":box2d",
        ":euler_angles_zxy",
        ":integral",
        ":kalman_filter",
        ":line_segment2d",
        ":linear_interpolation",
        ":lqr",
        ":math_utils",
        ":mpc",
        ":polygon2d",
        ":quaternion",
        ":search",
        ":sin_table",
        ":vec2d",
        ":factorial",
    ],
)

cc_library(
    name = "math_utils",
    srcs = [
        "math_utils.cc",
    ],
    hdrs = [
        "math_utils.h",
    ],
    deps = [
        ":vec2d",
        "//modules/common:log",
    ],
)

cc_library(
    name = "vec2d",
    srcs = [
        "vec2d.cc",
    ],
    hdrs = [
        "vec2d.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/util:string_util",
    ],
)

cc_library(
    name = "aabox2d",
    srcs = [
        "aabox2d.cc",
    ],
    hdrs = [
        "aabox2d.h",
    ],
    deps = [
        ":math_utils",
        ":vec2d",
        "//modules/common:log",
    ],
)

cc_library(
    name = "box2d",
    srcs = [
        "box2d.cc",
    ],
    hdrs = [
        "box2d.h",
    ],
    deps = [
        ":aabox2d",
        ":line_segment2d",
        ":math_utils",
        ":polygon2d",
        ":vec2d",
        "//modules/common:log",
    ],
)

cc_library(
    name = "sin_table",
    srcs = [
        "sin_table.cc",
    ],
    hdrs = [
        "sin_table.h",
    ],
)

cc_library(
    name = "angle",
    srcs = [
        "angle.cc",
    ],
    hdrs = [
        "angle.h",
    ],
    deps = [
        ":sin_table",
    ],
)

cc_library(
    name = "euler_angles_zxy",
    hdrs = [
        "euler_angles_zxy.h",
    ],
    deps = [
        ":math_utils",
        ":vec2d",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "aaboxkdtree2d",
    hdrs = [
        "aaboxkdtree2d.h",
    ],
    deps = [
        ":aabox2d",
        ":math_utils",
        ":vec2d",
        "//modules/common:log",
    ],
)

cc_library(
    name = "quaternion",
    hdrs = [
        "quaternion.h",
    ],
    deps = [
        ":euler_angles_zxy",
        ":math_utils",
        ":vec2d",
        "//modules/common/proto:common_proto",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "matrix_operations",
    hdrs = [
        "matrix_operations.h",
    ],
    deps = [
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "kalman_filter",
    hdrs = [
        "kalman_filter.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/math:matrix_operations",
        "//modules/common/util:string_util",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "factorial",
    hdrs = [
        "factorial.h",
    ],
    deps = [
    ],
)

cc_test(
    name = "angle_test",
    size = "small",
    srcs = [
        "angle_test.cc",
    ],
    deps = [
        ":angle",
        "@gtest//:main",
    ],
)

cc_library(
    name = "line_segment2d",
    srcs = [
        "line_segment2d.cc",
    ],
    hdrs = [
        "line_segment2d.h",
    ],
    deps = [
        "math_utils",
        ":vec2d",
        "//modules/common:log",
    ],
)

cc_library(
    name = "polygon2d",
    srcs = [
        "box2d.cc",
        "polygon2d.cc",
    ],
    hdrs = [
        "box2d.h",
        "polygon2d.h",
    ],
    deps = [
        ":aabox2d",
        ":line_segment2d",
        ":math_utils",
        ":vec2d",
        "//modules/common:log",
    ],
)

cc_library(
    name = "search",
    srcs = [
        "search.cc",
    ],
    hdrs = [
        "search.h",
    ],
    deps = [
    ],
)

cc_library(
    name = "linear_interpolation",
    srcs = [
        "linear_interpolation.cc",
    ],
    hdrs = [
        "linear_interpolation.h",
    ],
    deps = [
        ":math_utils",
    ],
)

cc_library(
    name = "integral",
    srcs = [
        "integral.cc",
    ],
    hdrs = [
        "integral.h",
    ],
    deps = [
        "//modules/common:log",
    ],
)

cc_library(
    name = "lqr",
    srcs = [
        "linear_quadratic_regulator.cc",
    ],
    hdrs = [
        "linear_quadratic_regulator.h",
    ],
    deps = [
        "//modules/common:log",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "mpc",
    srcs = [
        "mpc_solver.cc",
    ],
    hdrs = [
        "mpc_solver.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/math/qp_solver",
        "//modules/common/math/qp_solver:active_set_qp_solver",
        "@eigen//:eigen",
    ],
)

cc_test(
    name = "mpc_test",
    size = "small",
    srcs = [
        "mpc_solver_test.cc",
    ],
    deps = [
        ":mpc",
        "//modules/common/math/qp_solver",
        "//modules/common/math/qp_solver:active_set_qp_solver",
        "@gtest//:main",
    ],
)

cc_test(
    name = "math_utils_test",
    size = "small",
    srcs = [
        "math_utils_test.cc",
    ],
    deps = [
        ":math_utils",
        "@gtest//:main",
    ],
)

cc_test(
    name = "linear_interpolation_test",
    size = "small",
    srcs = [
        "linear_interpolation_test.cc",
    ],
    deps = [
        ":linear_interpolation",
        "//modules/common:log",
        "@eigen//:eigen",
        "@gtest//:main",
    ],
)

cc_test(
    name = "search_test",
    size = "small",
    srcs = [
        "search_test.cc",
    ],
    deps = [
        ":search",
        "@gtest//:main",
    ],
)

cc_test(
    name = "integral_test",
    size = "small",
    srcs = [
        "integral_test.cc",
    ],
    deps = [
        ":integral",
        "@gtest//:main",
    ],
)

cc_test(
    name = "vec2d_test",
    size = "small",
    srcs = [
        "vec2d_test.cc",
    ],
    deps = [
        ":vec2d",
        "@gtest//:main",
    ],
)

cc_test(
    name = "quaternion_test",
    size = "small",
    srcs = [
        "quaternion_test.cc",
    ],
    deps = [
        ":quaternion",
        "@gtest//:main",
    ],
)

cc_test(
    name = "euler_angles_zxy_test",
    size = "small",
    srcs = [
        "euler_angles_zxy_test.cc",
    ],
    deps = [
        ":euler_angles_zxy",
        "@gtest//:main",
    ],
)

cc_test(
    name = "aabox2d_test",
    size = "small",
    srcs = [
        "aabox2d_test.cc",
    ],
    deps = [
        ":aabox2d",
        "@gtest//:main",
    ],
)

cc_test(
    name = "aaboxkdtree2d_test",
    size = "small",
    srcs = [
        "aaboxkdtree2d_test.cc",
    ],
    deps = [
        ":aaboxkdtree2d",
        ":line_segment2d",
        ":math_utils",
        "@gtest//:main",
    ],
)

cc_test(
    name = "box2d_test",
    size = "small",
    srcs = [
        "box2d_test.cc",
    ],
    deps = [
        ":box2d",
        ":math_utils",
        "@gtest//:main",
    ],
)

cc_test(
    name = "polygon2d_test",
    size = "small",
    srcs = [
        "polygon2d_test.cc",
    ],
    deps = [
        ":box2d",
        ":line_segment2d",
        ":math_utils",
        ":polygon2d",
        "@gtest//:main",
    ],
)

cc_test(
    name = "line_segment2d_test",
    size = "small",
    srcs = [
        "line_segment2d_test.cc",
    ],
    deps = [
        ":line_segment2d",
        "@gtest//:main",
    ],
)

cc_test(
    name = "kalman_filter_test",
    size = "small",
    srcs = [
        "kalman_filter_test.cc",
    ],
    deps = [
        ":kalman_filter",
        "@gtest//:main",
    ],
)

cpplint()
