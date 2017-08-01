load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "spiral_curve",
    srcs = [
        "spiral_curve.cc",
    ],
    hdrs = [
        "spiral_curve.h",
    ],
    deps = [
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/status",
        "//modules/planning/proto:spiral_curve_config_proto",
    ],
)

cc_library(
    name = "spiral_formula",
    srcs = [
        "spiral_formula.cc",
    ],
    hdrs = [
        "spiral_formula.h",
    ],
    deps = [
    ],
)

cc_library(
    name = "cubic_sprial_curve",
    srcs = [
        "cubic_spiral_curve.cc",
    ],
    hdrs = [
        "cubic_spiral_curve.h",
    ],
    deps = [
        ":spiral_curve",
        ":spiral_formula",
        "//modules/common:log",
        "//modules/common/math",
        "//modules/common/proto:error_code_proto",
        "//modules/common/proto:pnc_point_proto",
        "@eigen//:eigen",
    ],
)

cc_library(
    name = "quintic_sprial_curve",
    srcs = [
        "quintic_spiral_curve.cc",
    ],
    hdrs = [
        "quintic_spiral_curve.h",
    ],
    deps = [
        ":spiral_curve",
        ":spiral_formula",
        "//modules/common:log",
        "//modules/common/math",
        "//modules/common/proto:pnc_point_proto",
        "@eigen//:eigen",
    ],
)

cpplint()
