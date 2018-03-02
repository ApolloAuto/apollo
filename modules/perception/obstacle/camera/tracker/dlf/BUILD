load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dlf",
    srcs = ["dlf_affinity_tracker.cc"],
    hdrs = ["dlf_affinity_tracker.h"],
    deps = [
        "@eigen",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "//modules/perception/obstacle/camera/tracker:base_affinity"
    ]
)

cpplint()
