load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "opengl_visualizer",
    srcs = [
        "camera.cc",
        "frame_content.cc",
        "glfw_viewer.cc",
        "opengl_visualizer.cc",
    ],
    hdrs = [
        "arc_ball.h",
        "camera.h",
        "frame_content.h",
        "glfw_viewer.h",
        "opengl_visualizer.h",
    ],
    copts = [
        "-Wno-deprecated",
    ],
    deps = [
        "//modules/common:log",
        "//modules/perception/common:pcl_util",
        "//modules/perception/lib/base",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/common",
        "@eigen",
        "@glew",
        "@glfw",
        "@pcl",
        "@vtk",
    ],
)

cpplint()
