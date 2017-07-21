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
    ],
)

cpplint()
