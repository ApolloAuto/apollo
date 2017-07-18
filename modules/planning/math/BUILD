load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "double",
    srcs = [
        "double.cc",
    ],
    hdrs = glob([
        "double.h",
    ]),
    deps = [
        "@glog//:glog",
    ],
)

cc_library(
    name = "curve_math",
    srcs = [
        "curve_math.cc",
    ],
    hdrs = glob([
        "curve_math.h",
    ]),
    deps = [
    ],
)

cc_library(
    name = "integration",
    srcs = [
        "integration.cc",
    ],
    hdrs = glob([
        "integration.h",
    ]),
    deps = [
        "@glog//:glog",
    ],
)

cc_library(
    name = "polynomial_xd",
    srcs = [
        "polynomial_xd.cc",
    ],
    hdrs = glob([
        "polynomial_xd.h",
    ]),
    deps = [
        "@glog//:glog",
    ],
)

cc_library(
    name = "sl_analytic_transformation",
    srcs = [
        "sl_analytic_transformation.cc",
    ],
    hdrs = glob([
        "sl_analytic_transformation.h",
    ]),
    deps = [
        ":double",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "interpolation",
    hdrs = glob([
        "interpolation.h",
    ]),
    deps = [
        "@glog//:glog",
    ],
)

cc_library(
    name = "hermite_spline",
    hdrs = glob([
        "hermite_spline.h",
    ]),
    deps = [
        "@glog//:glog",
    ],
)

cpplint()
