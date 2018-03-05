load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    srcs = [
    ],
    hdrs = [
        "caffe_bridge.hpp",
        "camera.h",
        "lane_object.h",
        "util.h",
        "visual_object.h",
    ],
    deps = [
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "@eigen",
        "@undistort",
    ],
)

cpplint()
