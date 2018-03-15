load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "visualizer",
    srcs = [
        "frame_content.cc",
        "gl_fusion_visualizer.cc",
        "glfw_fusion_viewer.cc",
    ],
    hdrs = [
        "base_visualizer.h",
        "frame_content.h",
        "gl_fusion_visualizer.h",
        "glfw_fusion_viewer.h",
    ],
    copts = [
        "-Wno-deprecated",
    ],
    deps = [
        "//modules/common:log",
        "//modules/drivers/proto:sensor_proto",
        "//modules/perception/common:pcl_util",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/camera/visualizer/common",
        "//modules/perception/obstacle/common",
        "//modules/perception/obstacle/radar/interface",
        "@eigen",
        "@glew",
        "@glfw",
        "@opengl"
    ],
)

cpplint()
