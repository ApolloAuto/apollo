load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")

apollo_cc_library(
    name = "math",
    srcs = [
        "aabox2d.cc",
        "angle.cc",
        "box2d.cc",
        "cartesian_frenet_conversion.cc",
        "integral.cc",
        "line_segment2d.cc",
        "linear_interpolation.cc",
        "linear_quadratic_regulator.cc",
        "math_utils.cc",
        "matrix_operations.cc",
        "mpc_osqp.cc",
        "path_matcher.cc",
        "polygon2d.cc",
        "search.cc",
        "sin_table.cc",
        "vec2d.cc",
    ],
    hdrs = [
        "aabox2d.h",
        "aaboxkdtree2d.h",
        "angle.h",
        "box2d.h",
        "cartesian_frenet_conversion.h",
        "curve_fitting.h",
        "euler_angles_zxy.h",
        "factorial.h",
        "hermite_spline.h",
        "integral.h",
        "kalman_filter.h",
        "line_segment2d.h",
        "linear_interpolation.h",
        "linear_quadratic_regulator.h",
        "math_utils.h",
        "matrix_operations.h",
        "mpc_osqp.h",
        "path_matcher.h",
        "polygon2d.h",
        "quaternion.h",
        "search.h",
        "sin_table.h",
        "vec2d.h",
    ],
    linkopts = ["-lm"],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/common/math/qp_solver",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "@com_google_absl//:absl",
        "@eigen",
        "@osqp",
    ],
)

apollo_cc_test(
    name = "matrix_operations_test",
    size = "small",
    srcs = ["matrix_operations_test.cc"],
    linkopts = ["-lm"],
    deps = [
        ":math",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "angle_test",
    size = "small",
    srcs = ["angle_test.cc"],
    linkopts = [
        "-lm",
    ],
    deps = [
        ":math",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "mpc_osqp_test",
    size = "small",
    srcs = ["mpc_osqp_test.cc"],
    deps = [
        ":math",
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "math_utils_test",
    size = "small",
    srcs = ["math_utils_test.cc"],
    deps = [
        ":math",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "linear_interpolation_test",
    size = "small",
    srcs = ["linear_interpolation_test.cc"],
    deps = [
        ":math",
        "//cyber",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

apollo_cc_test(
    name = "search_test",
    size = "small",
    srcs = ["search_test.cc"],
    linkopts = [
        "-lm",
    ],
    deps = [
        ":math",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "integral_test",
    size = "small",
    srcs = ["integral_test.cc"],
    linkopts = [
        "-lm",
    ],
    linkstatic = True,
    deps = [
        ":math",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "vec2d_test",
    size = "small",
    srcs = ["vec2d_test.cc"],
    deps = [
        ":math",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "quaternion_test",
    size = "small",
    srcs = ["quaternion_test.cc"],
    deps = [
        ":math",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "euler_angles_zxy_test",
    size = "small",
    srcs = ["euler_angles_zxy_test.cc"],
    linkstatic = True,
    deps = [
        ":math",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "aabox2d_test",
    size = "small",
    srcs = ["aabox2d_test.cc"],
    deps = [
        ":math",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "aaboxkdtree2d_test",
    size = "small",
    srcs = ["aaboxkdtree2d_test.cc"],
    deps = [
        ":math",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "box2d_test",
    size = "small",
    srcs = ["box2d_test.cc"],
    deps = [
        ":math",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "polygon2d_test",
    size = "small",
    srcs = ["polygon2d_test.cc"],
    deps = [
        ":math",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "line_segment2d_test",
    size = "small",
    srcs = ["line_segment2d_test.cc"],
    linkstatic = True,
    deps = [
        ":math",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "kalman_filter_test",
    size = "small",
    srcs = ["kalman_filter_test.cc"],
    linkopts = ["-lm"],
    linkstatic = True,
    deps = [
        ":math",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "cartesian_frenet_conversion_test",
    size = "small",
    srcs = ["cartesian_frenet_conversion_test.cc"],
    deps = [
        ":math",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
