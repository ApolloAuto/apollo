load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "pointcloud_map_based_roi_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
        "launch/**",
    ])
)

cc_binary(
    name = "libpointcloud_map_based_roi_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":pointcloud_map_based_roi_component_lib"],
)

cc_library(
    name = "pointcloud_map_based_roi_component_lib",
    srcs = ["pointcloud_map_based_roi_component.cc"],
    hdrs = ["pointcloud_map_based_roi_component.h"],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        "//cyber",
        "//modules/perception/common/lidar/common",
        "//modules/perception/common/lidar/scene_manager",
        "//modules/perception/common/onboard/inner_component_messages:lidar_inner_component_messages",
        "//modules/perception/pointcloud_map_based_roi/interface:base_roi_filter",
        "//modules/perception/pointcloud_map_based_roi/map_manager:map_manager",
        "//modules/perception/pointcloud_map_based_roi/proto:pointcloud_map_based_roi_component_config_cc_proto",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
