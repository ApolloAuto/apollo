load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "camera_detection_3d_files",
    srcs = glob([
        "conf/**",
        "dag/**",
    ])
)

cc_binary(
    name = "libcamera_detection_3d_component_camera.so",
    linkshared = True,
    linkstatic = True,
    deps = [":camera_detection_3d_component"],
)

cc_library(
    name = "camera_detection_3d_component",
    srcs = [
        "camera_detection_3d_component.cc",
    ],
    hdrs = [
        "camera_detection_3d_component.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler(),
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/perception/common/algorithm/i_lib/core",
        "//modules/perception/camera_detection_3d/interface:base_obstacle_detector",
        "//modules/perception/common/algorithm/sensor_manager",
        "//modules/perception/common/base",
        "//modules/perception/common:util",
        "//modules/perception/common/camera/common:data_provider",
        "//modules/perception/common/camera/common:util",
        "//modules/perception/common/onboard/transform_wrapper",
        "//modules/perception/common/onboard/inner_component_messages:camera_detection_component_messages",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/perception/camera_detection_3d/proto:camera_detection_3d_cc_proto",
        "//modules/perception/camera_detection_3d/detector/smoke:smoke_obstacle_detector",
        "//modules/perception/camera_detection_3d/detector/caddn:caddn_obstacle_detector",
    ]
)

apollo_package()
cpplint()
