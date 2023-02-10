load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    runtime_dest = "perception/bin",
    targets = [":offline_obstacle_pipeline"],
)

cc_library(
    name = "visualizer",
    srcs = ["visualizer.cc"],
    hdrs = [
        "colormap.h",
        "keycode.h",
        "visualizer.h",
    ],
    deps = [
        ":transform_server",
        "//modules/common/util:util_tool",
        "//modules/perception/camera/app:cipv_camera",
        "//modules/perception/camera/common",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/proto:motion_service_cc_proto",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
        "@opencv//:highgui",
    ],
)

cc_library(
    name = "transform_server",
    srcs = ["transform_server.cc"],
    hdrs = ["transform_server.h"],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/common/util:util_tool",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@eigen",
    ],
)

cc_binary(
    name = "offline_obstacle_pipeline",
    srcs = ["offline_obstacle_pipeline.cc"],
    linkstatic = True,
    linkopts = ["-lleveldb"],
    deps = [
        ":transform_server",
        ":visualizer",
        "//modules/perception/base",
        "//modules/perception/camera/app:obstacle_camera_perception",
        "//modules/perception/camera/app/proto:perception_cc_proto",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/calibration_service/online_calibration_service",
        "//modules/perception/camera/lib/lane/detector/darkSCNN:darkSCNN_lane_detector",
        "//modules/perception/camera/lib/lane/detector/denseline:denseline_lane_detector",
        "//modules/perception/camera/lib/lane/postprocessor/darkSCNN:darkSCNN_lane_postprocessor",
        "//modules/perception/camera/lib/lane/postprocessor/denseline:denseline_lane_postprocessor",
        "//modules/perception/common/sensor_manager",
        "//modules/perception/pipeline/proto/stage:tracking_feature_cc_proto",
        "@boost",
        "@com_google_absl//:absl",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
        "@opencv//:highgui",
    ],
)

cpplint()
