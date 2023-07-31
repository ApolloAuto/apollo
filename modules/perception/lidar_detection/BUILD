load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "lidar_detection_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
        "launch/**",
    ])
)

cc_binary(
    name = "liblidar_detection_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":lidar_detection_component_lib"],
)

cc_library(
    name = "lidar_detection_component_lib",
    srcs = ["lidar_detection_component.cc"],
    hdrs = ["lidar_detection_component.h"],
    copts = PERCEPTION_COPTS + if_profiler(),
    linkopts = ["-lleveldb"],
    deps = [
        "//cyber",
        "//modules/perception/common/onboard/inner_component_messages:lidar_inner_component_messages",
        "//modules/perception/lidar_detection/proto:lidar_detection_component_config_cc_proto",
        "//modules/perception/lidar_detection/interface:base_lidar_detector",
        "//modules/perception/lidar_detection/object_builder:object_builder",
        "//modules/perception/lidar_detection/detector/cnn_segmentation:cnn_segmentation",
        "//modules/perception/lidar_detection/detector/center_point_detection:center_point_detection",
        "//modules/perception/lidar_detection/detector/point_pillars_detection:point_pillars_detection",
        "//modules/perception/lidar_detection/detector/mask_pillars_detection:mask_pillars_detection",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
