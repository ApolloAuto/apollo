load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "localization_msf_common_util",
    srcs = [
        "compression.cc",
        "system_utility.cc",
        "frame_transform.cc",
    ],
    hdrs = [
        "compression.h",
        "extract_ground_plane.h",
        "rect2d.h",
        "system_utility.h",
        "threadpool.h",
        "voxel_grid_covariance_hdmap.h",
        "frame_transform.h",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common:macro",
        "@eigen//:eigen",
        "@gtest//:gtest",
    ],
)

cpplint()
