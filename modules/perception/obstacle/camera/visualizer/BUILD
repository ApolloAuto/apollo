load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_obstacle_camera_visualizer",
    srcs = [
        "frame_content.cc",
        "glfw_fusion_viewer.cc",
        "gl_fusion_visualizer.cc",
    ],
    hdrs = [
        "frame_content.h",
        "base_visualizer.h",
        "glfw_fusion_viewer.h",
        "gl_fusion_visualizer.h",
    ],
    copts = [
        "-Wno-deprecated",
    ],
    deps = [
        "//modules/common:log",
        "//modules/drivers/proto:sensor_proto",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "//modules/perception/obstacle/common:perception_obstacle_common",
        "//modules/perception/obstacle/radar/interface:perception_obstacle_radar_interface",
        "//modules/perception/obstacle/camera/visualizer/common:perception_obstacle_camera_visualizer_common",
        "@eigen//:eigen",
        "@glew//:glew",
        "@glfw//:glfw",
    ],
)

cpplint()
