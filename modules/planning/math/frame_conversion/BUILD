load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cartesian_frenet_conversion",
    srcs = [
        "cartesian_frenet_conversion.cc",
    ],
    hdrs = [
        "cartesian_frenet_conversion.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/math:math_utils",
        "//modules/common/math:vec2d",
        "//modules/planning/math:double",
        "@eigen//:eigen",
    ],
)

cc_test(
    name = "cartesian_frenet_conversion_test",
    size = "small",
    srcs = [
        "cartesian_frenet_conversion_test.cc",
    ],
    deps = [
        ":cartesian_frenet_conversion",
        "@gtest//:main",
    ],
)

cpplint()
