load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "pointcloud_ground_detection_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
        "launch/**",
    ])
)

cc_binary(
    name = "libpointcloud_ground_detection_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":pointcloud_ground_detection_component_lib"],
)

cc_library(
    name = "pointcloud_ground_detection_component_lib",
    srcs = ["pointcloud_ground_detection_component.cc"],
    hdrs = ["pointcloud_ground_detection_component.h"],
    copts = PERCEPTION_COPTS + if_profiler(),
    linkopts = ["-lleveldb"],
    deps = [
        "//cyber",
        "//modules/perception/common/onboard/inner_component_messages:lidar_inner_component_messages",
        "//modules/perception/pointcloud_ground_detection/proto:pointcloud_ground_detection_component_config_cc_proto",
        "//modules/perception/pointcloud_ground_detection/ground_detector/spatio_temporal_ground_detector:spatio_temporal_ground_detector",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
