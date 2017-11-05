load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "trajectory_stitcher",
    srcs = [
        "trajectory_stitcher.cc",
    ],
    hdrs = [
        "trajectory_stitcher.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/planning/common:frame",
        "//modules/planning/common:planning_gflags",
    ],
)

cpplint()
