load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_lidar_opengl_visualizer",
    srcs = [
        "camera.cc",
        "frame.cc",
        "glfw_viewer.cc",
        "opengl_visualizer.cc",
        "frame_content.cc",
        "pcl_vis_util.cc",
        ],
    hdrs = [
        "frame.h",
        "camera.h",
        "glfw_viewer.h",
        "opengl_visualizer.h",
        "frame_content.h",
        "pcl_vis_util.h",
        ],
    deps = [
        "//modules/perception/lib/pcl_util:pcl_util",
        "//modules/common:log",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "//modules/perception/lib/base:base",
        "//modules/perception/obstacle/common:perception_obstacle_common",
        "@eigen//:eigen",
        "@pcl//:pcl",
        "@vtk//:vtk",
        "@glew//:glew",
        "@glfw//:glfw",
    ],
)

cpplint()
