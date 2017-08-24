load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_lidar_opengl_visualizer",
    srcs = [
        "camera.cc",
        "frame_content.cc",
        "glfw_viewer.cc",
        "opengl_visualizer.cc",
    ],
    hdrs = [
        "camera.h",
        "frame_content.h",
        "glfw_viewer.h",
        "opengl_visualizer.h",
	    "arc_ball.h",
    ],
    copts = [
        "-Wno-deprecated",
    ],
    deps = [
        "//modules/common:log",
        "//modules/perception/lib/base",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "//modules/perception/obstacle/common:perception_obstacle_common",
        "@eigen//:eigen",
        "@glew//:glew",
        "@glfw//:glfw",
        "@pcl//:pcl",
        "@vtk//:vtk",
    ],
)

cpplint()
