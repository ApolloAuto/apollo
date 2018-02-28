load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_camera_visualizer_common",
    srcs = [
        "camera.cc",
        "frame.cc",
        "gl_raster_text.cc"
    ],
    hdrs = [
        "bmp.h",
        "camera.h",
        "frame.h",
        "gl_raster_text.h"
    ],
    copts = [
        "-Wno-deprecated",
    ],
    deps = [
        "@eigen//:eigen",
        "@glew//:glew",
        "@glfw//:glfw"
    ],
)

cpplint()
