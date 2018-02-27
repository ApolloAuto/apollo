load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_camera_tracker",
    srcs = [
        "mix_camera_tracker_util.cc"
    ],
    hdrs =  glob(["*.h"]),
    deps = [
        "//modules/perception/lib/base",
        "//modules/perception/obstacle/common:perception_obstacle_common",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "//modules/perception/obstacle/camera/common:perception_obstacle_camera_common",
        "@eigen"
    ],
)

cpplint()
