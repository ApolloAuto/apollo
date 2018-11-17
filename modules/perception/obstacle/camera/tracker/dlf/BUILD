load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dlf",
    srcs = ["dlf_affinity_tracker.cc"],
    hdrs = ["dlf_affinity_tracker.h"],
    deps = [
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/camera/tracker:base_affinity",
        "@eigen",
    ],
)

cpplint()
