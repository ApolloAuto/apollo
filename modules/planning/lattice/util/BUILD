load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "reference_line_frame_converter",
    srcs = [
        "reference_line_frame_converter.cc",
    ],
    hdrs = [
        "reference_line_frame_converter.h",
    ],
    deps = [
        "//modules/common",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/math:cartesian_frenet_conversion",
        "//modules/common/math:path_matcher",
    ],
)

cc_library(
    name = "lattice_trajectory1d",
    srcs = [
        "lattice_trajectory1d.cc",
    ],
    hdrs = [
        "lattice_trajectory1d.h",
    ],
    deps = [
        "//modules/planning/math/curve1d",
    ],
)

cpplint()
