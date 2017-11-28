load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "standing_still_trajectory1d",
    hdrs = [
        "standing_still_trajectory1d.h",
    ],
    srcs = [
        "standing_still_trajectory1d.cc",
    ],
    deps = [
        "//modules/planning/math/curve1d:curve1d",
    ],
)

cc_library(
    name = "piecewise_acceleration_trajectory1d",
    hdrs = [
        "piecewise_acceleration_trajectory1d.h",
    ],
    srcs = [
        "piecewise_acceleration_trajectory1d.cc",
    ],
    deps = [
        "//modules/common:common",
        "//modules/common/math:linear_interpolation",
        "//modules/planning/math/curve1d:curve1d",
    ],
)

cc_library(
    name = "constant_deceleration_trajectory1d",
    hdrs = [
        "constant_deceleration_trajectory1d.h",
    ],
    srcs = [
        "constant_deceleration_trajectory1d.cc",
    ],
    deps = [
        "//modules/common:common",
        "//modules/planning/math/curve1d:curve1d",
    ],
)

cc_library(
    name = "piecewise_trajectory1d",
    hdrs = [
        "piecewise_trajectory1d.h",
    ],
    srcs = [
        "piecewise_trajectory1d.cc",
    ],
    deps = [
        "//modules/common:common",
        "//modules/common/math:linear_interpolation",
        "//modules/planning/math/curve1d:curve1d",
    ],
)

cc_library(
    name = "lattice_trajectory1d",
    deps = [
        ":standing_still_trajectory1d",
        ":piecewise_acceleration_trajectory1d",
        ":constant_deceleration_trajectory1d",
        ":piecewise_trajectory1d",
    ],
)

cpplint()
