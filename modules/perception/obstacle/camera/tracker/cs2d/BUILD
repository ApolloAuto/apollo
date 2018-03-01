load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_camera_tracker_cs2d",
    srcs = ["cs2d_affinity_tracker.cc"],
    hdrs =  glob(["*.h"]),
    deps = [
        "//modules/perception/obstacle/camera/tracker:camera_tracker_cascaded"
    ]
)

cpplint()
