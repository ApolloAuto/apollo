load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "obstacle_camera_interface",
    srcs = [],
    hdrs = glob(["*.h"]),
    deps = [
        "//modules/perception/lib/base:base",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "//modules/perception/obstacle/camera/common:obstacle_camera_common",
        "@eigen",
        "@opencv2//:core"
    ]
)

cpplint()
