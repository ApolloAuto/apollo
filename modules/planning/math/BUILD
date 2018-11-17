load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "curve_math",
    srcs = [
        "curve_math.cc",
    ],
    hdrs = [
        "curve_math.h",
    ],
)

cc_test(
    name = "curve_math_test",
    size = "small",
    srcs = [
        "curve_math_test.cc",
    ],
    deps = [
        ":curve_math",
        "@gtest//:main",
    ],
)

cc_library(
    name = "polynomial_xd",
    srcs = [
        "polynomial_xd.cc",
    ],
    hdrs = [
        "polynomial_xd.h",
    ],
    deps = [
        "//modules/common:log",
    ],
)

cpplint()
