load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "kcf",
    srcs = [
        "kcf_affinity_tracker.cc",
        "kcf_components.cc"
    ],
    hdrs = [
        "kcf_affinity_tracker.h",
        "kcf_components.h"
    ],
    deps = [
        "@eigen//:eigen",
        "@opencv2//:core",
        "//modules/perception/lib/base",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "//modules/perception/obstacle/common:perception_obstacle_common",
        "//modules/perception/obstacle/camera/common:common",
        "//modules/perception/obstacle/camera/tracker:base_affinity",
        "//modules/perception/obstacle/camera/tracker:util"
    ]
)

cpplint()
