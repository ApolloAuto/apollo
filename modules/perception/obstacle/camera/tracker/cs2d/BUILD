load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "cs2d",
    srcs = ["cs2d_affinity_tracker.cc"],
    hdrs = ["cs2d_affinity_tracker.h"],
    deps = [
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/camera/tracker:base_affinity",
    ],
)

cpplint()
