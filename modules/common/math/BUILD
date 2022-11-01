load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

install(
    name = "install",
    library_dest = "common/lib",
    data_dest = "common",
    runtime_dest = "common/bin",
    targets = [
        ":libmath.so",
    ],
    visibility = ["//visibility:public"],
)


cc_binary(
    name = "libmath.so",
    linkshared = True,
    linkstatic = True,
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
        ":matrix_operations",
        ":mpc_osqp",
        ":path_matcher",
        ":quaternion",
        ":search",
        ":sin_table",
        "//modules/common/math/qp_solver",
    ],
)

cc_library(
    name = "math",
    srcs = ["libmath.so"],
    hdrs = glob(["*.h"]),
    deps = [
        "//cyber",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/util:util_tool",
        "@com_google_absl//:absl",
        "@eigen",
        "@osqp",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "math_utils",
    srcs = ["math_utils.cc"],
    hdrs = ["math_utils.h"],
    alwayslink = True,
    linkopts = ["-lm"],
    deps = [
        ":vec2d",
        "@eigen",
    ],
)

cc_library(
    name = "vec2d",
    srcs = ["vec2d.cc"],
    hdrs = ["vec2d.h"],
    alwayslink = True,
    linkopts = ["-lm"],
    deps = [
        "//cyber",
        "@com_google_absl//:absl",
    ],
)

cc_library(
    name = "geometry",
    srcs = [
        "aabox2d.cc",
        "box2d.cc",
        "line_segment2d.cc",
        "polygon2d.cc",
    ],
    hdrs = [
        "aabox2d.h",
        "aaboxkdtree2d.h",
        "box2d.h",
        "line_segment2d.h",
        "polygon2d.h",
    ],
    alwayslink = True,
    linkopts = ["-lm"],
    deps = [
        ":math_utils",
        "//cyber",
        "//modules/common/util:util_tool",
        "@com_google_absl//:absl",
        "@osqp",
    ],
)

cc_library(
    name = "sin_table",
    srcs = ["sin_table.cc"],
    hdrs = ["sin_table.h"],
    alwayslink = True,
)

cc_library(
    name = "angle",
    srcs = ["angle.cc"],
    hdrs = ["angle.h"],
    alwayslink = True,
    deps = [
        ":sin_table",
    ],
)

cc_library(
    name = "euler_angles_zxy",
    hdrs = ["euler_angles_zxy.h"],
    alwayslink = True,
    deps = [
        ":geometry",
        "@eigen",
    ],
)

cc_library(
    name = "quaternion",
    hdrs = ["quaternion.h"],
    alwayslink = True,
    deps = [
        ":euler_angles_zxy",
        ":geometry",
        "//modules/common_msgs/basic_msgs:geometry_cc_proto",
        "@eigen",
    ],
)

cc_library(
    name = "curve_fitting",
    hdrs = ["curve_fitting.h"],
    alwayslink = True,
    deps = [
        ":matrix_operations",
        "@eigen",
    ],
)

cc_library(
    name = "matrix_operations",
    srcs = ["matrix_operations.cc"],
    hdrs = ["matrix_operations.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "@eigen",
    ],
)

cc_test(
    name = "matrix_operations_test",
    size = "small",
    srcs = ["matrix_operations_test.cc"],
    linkopts = ["-lm"],
    deps = [
        ":matrix_operations",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "kalman_filter",
    hdrs = ["kalman_filter.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        ":matrix_operations",
        "@eigen",
    ],
)

cc_library(
    name = "factorial",
    hdrs = ["factorial.h"],
    alwayslink = True,
)

cc_library(
    name = "path_matcher",
    srcs = ["path_matcher.cc"],
    hdrs = ["path_matcher.h"],
    deps = [
        ":linear_interpolation",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
    ],
    alwayslink = True,
)

cc_library(
    name = "mpc_osqp",
    srcs = ["mpc_osqp.cc"],
    hdrs = ["mpc_osqp.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "@eigen",
        "@osqp",
    ],
)

cc_test(
    name = "angle_test",
    size = "small",
    srcs = ["angle_test.cc"],
    linkopts = [
        "-lm",
    ],
    deps = [
        ":angle",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "search",
    srcs = ["search.cc"],
    hdrs = ["search.h"],
    alwayslink = True,
)

cc_library(
    name = "linear_interpolation",
    srcs = ["linear_interpolation.cc"],
    hdrs = ["linear_interpolation.h"],
    alwayslink = True,
    deps = [
        ":geometry",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
    ],
)

cc_library(
    name = "integral",
    srcs = ["integral.cc"],
    hdrs = ["integral.h"],
    alwayslink = True,
    deps = [
        "//cyber",
    ],
)

cc_library(
    name = "lqr",
    srcs = ["linear_quadratic_regulator.cc"],
    hdrs = ["linear_quadratic_regulator.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "@eigen",
    ],
)

cc_library(
    name = "cartesian_frenet_conversion",
    srcs = ["cartesian_frenet_conversion.cc"],
    hdrs = ["cartesian_frenet_conversion.h"],
    alwayslink = True,
    deps = [
        ":geometry",
        "//cyber",
        "@eigen",
    ],
)

cc_library(
    name = "hermite_spline",
    hdrs = ["hermite_spline.h"],
    alwayslink = True,
    deps = [
        "//cyber",
    ],
)

cc_test(
    name = "mpc_osqp_test",
    size = "small",
    srcs = ["mpc_osqp_test.cc"],
    deps = [
        ":mpc_osqp",
        "//cyber",
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
        "//cyber",
        "@com_google_googletest//:gtest_main",
        "@eigen",
    ],
)

cc_test(
    name = "search_test",
    size = "small",
    srcs = ["search_test.cc"],
    linkopts = [
        "-lm",
    ],
    deps = [
        ":search",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "integral_test",
    size = "small",
    srcs = ["integral_test.cc"],
    linkopts = [
        "-lm",
    ],
    deps = [
        ":integral",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
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
    linkstatic = True,
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
    linkstatic = True,
)

cc_test(
    name = "kalman_filter_test",
    size = "small",
    srcs = ["kalman_filter_test.cc"],
    linkopts = ["-lm"],
    deps = [
        ":kalman_filter",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
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
