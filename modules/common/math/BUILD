load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "math",
    deps = [
        ":angle",
        ":cartesian_frenet_conversion",
        ":curve_fitting",
        ":euler_angles_zxy",
        ":factorial",
        ":geometry",
        ":integral",
        ":kalman_filter",
        ":linear_interpolation",
        ":lqr",
        ":mpc",
        ":quaternion",
        ":search",
        ":sin_table",
    ],
)

cc_library(
    name = "geometry",
    srcs = [
        "aabox2d.cc",
        "box2d.cc",
        "line_segment2d.cc",
        "math_utils.cc",
        "math_utils.h",
        "polygon2d.cc",
        "vec2d.cc",
    ],
    hdrs = [
        "aabox2d.h",
        "aaboxkdtree2d.h",
        "box2d.h",
        "line_segment2d.h",
        "polygon2d.h",
        "vec2d.h",
    ],
    deps = [
        "//cyber/common:log",
        "//modules/common/util:string_util",
        "@com_google_absl//absl/strings",
        "@eigen",
        "@osqp",
    ],
)

cc_library(
    name = "sin_table",
    srcs = ["sin_table.cc"],
    hdrs = ["sin_table.h"],
)

cc_library(
    name = "angle",
    srcs = ["angle.cc"],
    hdrs = ["angle.h"],
    deps = [
        ":sin_table",
    ],
)

cc_library(
    name = "euler_angles_zxy",
    hdrs = ["euler_angles_zxy.h"],
    deps = [
        ":geometry",
        "@eigen",
    ],
)

cc_library(
    name = "quaternion",
    hdrs = ["quaternion.h"],
    deps = [
        ":euler_angles_zxy",
        ":geometry",
        "//modules/common/proto:geometry_proto",
        "@eigen",
    ],
)

cc_library(
    name = "curve_fitting",
    hdrs = ["curve_fitting.h"],
    deps = [
        ":matrix_operations",
        "@eigen",
    ],
)

cc_library(
    name = "matrix_operations",
    srcs = ["matrix_operations.cc"],
    hdrs = ["matrix_operations.h"],
    deps = [
        "//cyber/common:log",
        "@eigen",
    ],
)

cc_test(
    name = "matrix_operations_test",
    size = "small",
    srcs = ["matrix_operations_test.cc"],
    deps = [
        ":matrix_operations",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "kalman_filter",
    hdrs = ["kalman_filter.h"],
    deps = [
        "//cyber/common:log",
        "//modules/common/math:matrix_operations",
        "@eigen",
    ],
)

cc_library(
    name = "factorial",
    hdrs = ["factorial.h"],
)

cc_library(
    name = "path_matcher",
    srcs = ["path_matcher.cc"],
    hdrs = ["path_matcher.h"],
    deps = [
        "//modules/common/math:linear_interpolation",
        "//modules/common/proto:pnc_point_proto",
    ],
)

cc_library(
    name = "mpc_osqp",
    srcs = ["mpc_osqp.cc"],
    hdrs = ["mpc_osqp.h"],
    deps = [
        "//cyber/common:log",
        "@eigen",
        "@osqp",
    ],
)

cc_test(
    name = "angle_test",
    size = "small",
    srcs = ["angle_test.cc"],
    deps = [
        ":angle",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "search",
    srcs = ["search.cc"],
    hdrs = ["search.h"],
)

cc_library(
    name = "linear_interpolation",
    srcs = ["linear_interpolation.cc"],
    hdrs = ["linear_interpolation.h"],
    deps = [
        ":geometry",
        "//modules/common/proto:pnc_point_proto",
    ],
)

cc_library(
    name = "integral",
    srcs = ["integral.cc"],
    hdrs = ["integral.h"],
    deps = [
        "//cyber/common:log",
    ],
)

cc_library(
    name = "lqr",
    srcs = ["linear_quadratic_regulator.cc"],
    hdrs = ["linear_quadratic_regulator.h"],
    deps = [
        "//cyber/common:log",
        "@eigen",
    ],
)

cc_library(
    name = "mpc",
    srcs = ["mpc_solver.cc"],
    hdrs = ["mpc_solver.h"],
    deps = [
        "//cyber/common:log",
        "//modules/common/math/qp_solver",
        "//modules/common/math/qp_solver:active_set_qp_solver",
        "@eigen",
    ],
)

cc_library(
    name = "cartesian_frenet_conversion",
    srcs = ["cartesian_frenet_conversion.cc"],
    hdrs = ["cartesian_frenet_conversion.h"],
    deps = [
        ":geometry",
        "//cyber/common:log",
        "@eigen",
    ],
)

cc_library(
    name = "hermite_spline",
    hdrs = ["hermite_spline.h"],
    deps = [
        "//cyber/common:log",
    ],
)

cc_test(
    name = "mpc_osqp_test",
    size = "small",
    srcs = ["mpc_osqp_test.cc"],
    deps = [
        ":mpc",
        ":mpc_osqp",
        "//modules/common/math/qp_solver",
        "//modules/common/math/qp_solver:active_set_qp_solver",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "mpc_test",
    size = "small",
    srcs = ["mpc_solver_test.cc"],
    deps = [
        ":mpc",
        "//modules/common/math/qp_solver",
        "//modules/common/math/qp_solver:active_set_qp_solver",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "math_utils_test",
    size = "small",
    srcs = ["math_utils_test.cc"],
    deps = [
        ":geometry",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "linear_interpolation_test",
    size = "small",
    srcs = ["linear_interpolation_test.cc"],
    deps = [
        ":linear_interpolation",
        "//cyber/common:log",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

cc_test(
    name = "search_test",
    size = "small",
    srcs = ["search_test.cc"],
    deps = [
        ":search",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "integral_test",
    size = "small",
    srcs = ["integral_test.cc"],
    deps = [
        ":integral",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "vec2d_test",
    size = "small",
    srcs = ["vec2d_test.cc"],
    deps = [
        ":geometry",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "quaternion_test",
    size = "small",
    srcs = ["quaternion_test.cc"],
    deps = [
        ":quaternion",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "euler_angles_zxy_test",
    size = "small",
    srcs = ["euler_angles_zxy_test.cc"],
    deps = [
        ":euler_angles_zxy",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "aabox2d_test",
    size = "small",
    srcs = ["aabox2d_test.cc"],
    deps = [
        ":geometry",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "aaboxkdtree2d_test",
    size = "small",
    srcs = ["aaboxkdtree2d_test.cc"],
    deps = [
        ":geometry",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "box2d_test",
    size = "small",
    srcs = ["box2d_test.cc"],
    deps = [
        ":geometry",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "polygon2d_test",
    size = "small",
    srcs = ["polygon2d_test.cc"],
    deps = [
        ":geometry",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "line_segment2d_test",
    size = "small",
    srcs = ["line_segment2d_test.cc"],
    deps = [
        ":geometry",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "kalman_filter_test",
    size = "small",
    srcs = ["kalman_filter_test.cc"],
    deps = [
        ":kalman_filter",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "cartesian_frenet_conversion_test",
    size = "small",
    srcs = ["cartesian_frenet_conversion_test.cc"],
    deps = [
        ":cartesian_frenet_conversion",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
