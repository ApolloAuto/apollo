load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pnc_trajectory",
    srcs = [
        "trajectory.cc",
    ],
    hdrs = [
        "trajectory.h",
    ],
    deps = [
        "//modules/map/proto:map_proto",
        "//modules/map/hdmap:hdmap",
        "//modules/common/math:math",
    ],
)

cpplint()
