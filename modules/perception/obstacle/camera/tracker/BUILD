load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "camera_tracker_cascaded",
    srcs =
        [
          "cascaded_camera_tracker.cc",
          "cascaded_camera_tracker_util.cc"
        ],
    hdrs =
        [
          "base_affinity_tracker.h",
          "cascaded_camera_tracker.h",
          "cascaded_camera_tracker_util.h"
        ],
    deps =
        [
          "//modules/common:log",
          "//modules/perception/lib/base",
          "//modules/perception/obstacle/base:perception_obstacle_base",
          "//modules/perception/obstacle/common:perception_obstacle_common",
          "//modules/perception/obstacle/camera/common:obstacle_camera_common",
          "@eigen"
        ], )

cpplint()
