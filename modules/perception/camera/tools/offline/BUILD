load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

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
        "//modules/perception/camera/app:cipv_camera",
        "//modules/perception/camera/common",
        "//modules/perception/onboard/proto:motion_service_proto",
        "//modules/perception/proto:perception_proto",
        "//third_party:opencv",
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
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@eigen",
    ],
)

cc_binary(
    name = "offline_obstacle_pipeline",
    srcs = ["offline_obstacle_pipeline.cc"],
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
        "//third_party:boost",
        "//third_party:opencv",
        "@com_google_absl//absl/strings",
    ],
)

cpplint()
