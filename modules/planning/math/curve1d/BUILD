load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "curve1d",
    hdrs = [
        "curve1d.h",
    ],
)

cc_library(
    name = "polynomial_curve1d",
    hdrs = [
        "polynomial_curve1d.h",
    ],
    deps = [
        ":curve1d",
    ],
)

cc_library(
    name = "quartic_polynomial_curve1d",
    srcs = [
        "quartic_polynomial_curve1d.cc",
    ],
    hdrs = [
        "quartic_polynomial_curve1d.h",
    ],
    deps = [
        ":polynomial_curve1d",
        "//modules/common:log",
        "//modules/common/util:string_util",
    ],
)

cc_library(
    name = "quintic_polynomial_curve1d",
    srcs = [
        "quintic_polynomial_curve1d.cc",
    ],
    hdrs = [
        "quintic_polynomial_curve1d.h",
    ],
    deps = [
        ":polynomial_curve1d",
        "//modules/common:log",
        "//modules/common/util:string_util",
    ],
)

cc_library(
    name = "quintic_spiral_path",
    srcs = [
        "quintic_spiral_path.cc",
    ],
    hdrs = [
        "quintic_spiral_path.h",
    ],
    deps = [
        ":quintic_polynomial_curve1d",
        "//modules/common:log",
        "//modules/common/math",
        "//modules/common/util:string_util",
    ],
)

cpplint()
