load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_affinity",
    srcs = [],
    hdrs = ["base_affinity_tracker.h"],
    deps = [
        "@opencv2//:core",
        "@eigen",
    ],
)

cc_library(
    name = "util",
    srcs = ["cascaded_camera_tracker_util.cc"],
    hdrs = ["cascaded_camera_tracker_util.h"],
    deps = [
        ":base_affinity",
        "//modules/common:log",
        "//modules/perception/lib/base",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/camera/common:util",
        "//modules/perception/obstacle/common",
        "@eigen",
    ],
)

cc_library(
    name = "tracker",
    srcs = ["cascaded_camera_tracker.cc"],
    hdrs = ["cascaded_camera_tracker.h"],
    deps = [
        ":base_affinity",
        ":util",
        "//modules/common:log",
        "//modules/perception/lib/base",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/camera/common",
        "//modules/perception/obstacle/camera/interface",
        "//modules/perception/obstacle/camera/tracker/cs2d",
        "//modules/perception/obstacle/camera/tracker/dlf",
        "//modules/perception/obstacle/camera/tracker/kcf",
        "//modules/perception/obstacle/common",
        "@eigen",
    ],
)

cpplint()
