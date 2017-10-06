load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "behavior_decider",
    hdrs = [
        "behavior_decider.h",
    ],
    srcs = [
        "behavior_decider.cc",
    ],
    deps = [
        "//modules/common/math:box2d",
        "//modules/planning/common/trajectory:discretized_trajectory",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
        "//modules/planning/lattice:planning_target",
        "//modules/planning/reference_line",
        "//modules/common/proto:pnc_point_proto",
        "//external:gflags",
    ],
)

cpplint()
