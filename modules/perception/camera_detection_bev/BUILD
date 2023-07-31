load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

cc_library(
    name = "camera_frame",
    hdrs = [
        "camera_frame.h",
    ],
    deps = [
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common:data_provider",
    ]
)

cc_library(
    name = "camera_detection_bev_component",
    srcs = [
        "camera_detection_bev_component.cc",
    ],
    hdrs = [
        "camera_detection_bev_component.h",
    ],
    copts = PERCEPTION_COPTS,
    deps = [
        ":camera_frame",
        "//cyber",
        "//modules/perception/common:util",
        "//modules/perception/camera_detection_bev/interface:base_obstacle_detector",
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common:data_provider",
        "//modules/perception/common/camera/common:util",
        "//modules/perception/common/onboard/transform_wrapper",
        "//modules/common_msgs/sensor_msgs:sensor_image_cc_proto",
        "//modules/perception/camera_detection_bev/proto:camera_detection_bev_cc_proto",
    ]
)

apollo_package()
cpplint()
