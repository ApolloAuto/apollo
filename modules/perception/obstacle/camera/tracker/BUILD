load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_affinity",
    srcs = [],
    hdrs = ["base_affinity_tracker.h"],
    deps = ["@opencv2//:core"]
)

cc_library(
    name = "util",
    srcs = ["cascaded_camera_tracker_util.cc"],
    hdrs = ["cascaded_camera_tracker_util.h"],
    deps = [
        "@eigen",
        "//modules/common:log",
        "//modules/perception/lib/base",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "//modules/perception/obstacle/common:perception_obstacle_common",
        "//modules/perception/obstacle/camera/common:common",
        ":base_affinity"
    ],
)

cc_library(
    name = "cascaded",
    srcs = ["cascaded_camera_tracker.cc",],
    hdrs = ["cascaded_camera_tracker.h",],
    deps = [
        "@eigen",
        "//modules/common:log",
        "//modules/perception/lib/base",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "//modules/perception/obstacle/common:perception_obstacle_common",
        "//modules/perception/obstacle/camera/common:common",
        "//modules/perception/obstacle/camera/interface:interface",
        "//modules/perception/obstacle/camera/tracker/cs2d:cs2d",
        "//modules/perception/obstacle/camera/tracker/dlf:dlf",
        "//modules/perception/obstacle/camera/tracker/kcf:kcf",
        ":base_affinity",
        ":util"
    ],
)

cpplint()
