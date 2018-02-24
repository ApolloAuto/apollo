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

cpplint()
