load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "trajectory_partitioner",
    srcs = ["trajectory_partitioner.cc"],
    hdrs = ["trajectory_partitioner.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        "//modules/planning/common:frame",
        "//modules/planning/common/trajectory:trajectory_stitcher",
    ],
)

cpplint()
