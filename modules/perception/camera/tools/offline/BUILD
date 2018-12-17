load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "visualizer",
    srcs = [
        "visualizer.cc",
    ],
    hdrs = [
        "visualizer.h",
    ],
    linkopts = [
        "-lopencv_core",
        "-lopencv_imgproc",
        "-lopencv_highgui",
    ],
    deps = [
        ":transform_server",
        "//modules/perception/camera/common",
    ],
)

cc_library(
    name = "transform_server",
    srcs = [
        "transform_server.cc",
    ],
    hdrs = [
        "transform_server.h",
    ],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "@eigen",
        "@yaml_cpp//:yaml",
    ],
)

cc_binary(
    name = "offline_obstacle_pipeline",
    srcs = [
        "offline_obstacle_pipeline.cc",
    ],
    linkopts = [
        "-lopencv_core",
        "-lopencv_imgproc",
        "-lopencv_highgui",
        "-lboost_system",
    ],
    deps = [
        ":transform_server",
        ":visualizer",
        "//modules/perception/base",
        "//modules/perception/camera/app:obstacle_camera_perception",
        "//modules/perception/camera/common",
        "//modules/perception/lib/utils",
    ],
)

cpplint()
