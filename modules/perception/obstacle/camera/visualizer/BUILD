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
    linkopts = ["-lGLU -lGL -lglut"],
    deps = [
        "//modules/common:log",
        "//modules/drivers/proto:sensor_proto",
        "//modules/perception/common:pcl_util",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/camera/lane_post_process/common:type",
        "//modules/perception/obstacle/camera/lane_post_process/common:util",
        "//modules/perception/obstacle/camera/visualizer/common",
        "//modules/perception/obstacle/common",
        "//modules/perception/obstacle/radar/interface",
        "//modules/perception/proto:lane_post_process_config_lib_proto",
        "@eigen",
        "@glew",
        "@glfw",
        "@pcl",
        "@vtk",
    ],
)

cc_binary(
    name = "camera_visualizer",
    srcs = [
        "camera_visualizer.cc",
    ],
    linkopts = ["-lopencv_core -lopencv_imgproc -lopencv_highgui"],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/common/adapters:adapter_gflags",
        "//modules/perception/common",
        "//modules/perception/lib/base",
        "//modules/perception/proto:perception_proto",
        "//modules/perception/traffic_light/base",
        "//modules/perception/traffic_light/util",
        "@ros//:ros_common",
    ],
)

cpplint()
