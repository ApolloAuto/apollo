load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "piecewise_poly_speed_curve",
    srcs = [
        "piecewise_poly_speed_curve.cc",
    ],
    hdrs = [
        "piecewise_poly_speed_curve.h",
    ],
    deps = [
        "//modules/common/proto:pnc_point_proto",
        "//modules/planning/math/curve1d:cubic_polynomial_curve1d",
        "//modules/planning/math/curve1d:quartic_polynomial_curve1d",
        "//modules/planning/math/curve1d:quintic_polynomial_curve1d",
    ],
)


cc_test(
    name = "piecewise_poly_speed_curve_test",
    size = "small",
    srcs = [
        "piecewise_poly_speed_curve_test.cc",
    ],
    deps = [
        ":piecewise_poly_speed_curve",
        "@gtest//:main",
    ],
)

cc_library(
    name = "piecewise_poly_speed_profile",
    srcs = [
        "piecewise_poly_speed_profile.cc",
    ],
    hdrs = [
        "piecewise_poly_speed_profile.h",
    ],
    deps = [
        "//modules/common/proto:pnc_point_proto",
        ":piecewise_poly_speed_curve",
    ],
)

cc_test(
    name = "piecewise_poly_speed_profile_test",
    size = "small",
    srcs = [
        "piecewise_poly_speed_profile_test.cc",
    ],
    deps = [
        ":piecewise_poly_speed_profile",
        "@gtest//:main",
    ],
)
cpplint()
