load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "visualizer",
    srcs = [
        "visualizer.cc",
    ],
    hdrs = [
        "keycode.h",
        "visualizer.h",
    ],
    linkopts = [
        "-lopencv_core",
        "-lopencv_imgproc",
        "-lopencv_highgui",
    ],
    deps = [
        ":transform_server",
        "//modules/perception/camera/app:cipv_camera",
        "//modules/perception/camera/common",
        "//modules/perception/onboard/proto:motion_service_proto",
        "//modules/perception/proto:perception_proto",
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
    linkstatic = False,
    deps = [
        ":transform_server",
        ":visualizer",
        "//modules/perception/base",
        "//modules/perception/camera/app:obstacle_camera_perception",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/lane/detector/darkSCNN:darkSCNN_lane_detector",
        "//modules/perception/camera/lib/lane/postprocessor/darkSCNN:darkSCNN_lane_postprocessor",
        "//modules/perception/lib/utils",
    ],
)

cpplint()
