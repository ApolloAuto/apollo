load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
  name = "tracker_affinity_kcf",
  srcs = [
    "kcf_affinity_tracker.cc",
    "kcf_components.cc"
  ],
  hdrs = [
    "kcf_affinity_tracker.h",
    "kcf_components.h"
  ],
  deps = [
    "//modules/perception/lib/base:base",
    "//modules/perception/obstacle/base:perception_obstacle_base",
    "//modules/perception/obstacle/common:perception_obstacle_common",
    "//modules/perception/obstacle/camera/common:obstacle_camera_common",
    "//modules/perception/obstacle/camera/tracker:camera_tracker_cascaded",
    "@eigen//:eigen",
    "@opencv2//:core",
  ]
)

cpplint()
