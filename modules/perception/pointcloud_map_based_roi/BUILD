load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
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
    ]),
)

apollo_cc_library(
    name = "apollo_perception_pointcloud_map_based_roi",
    srcs = [
        "map_manager/map_manager.cc",
    ],
    hdrs = [
        "interface/base_roi_filter.h",
        "map_manager/map_manager.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/map:apollo_map",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/hdmap:apollo_perception_common_hdmap",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/pointcloud_map_based_roi/map_manager/proto:map_manager_config_cc_proto",
        "//modules/perception/pointcloud_map_based_roi/proto:pointcloud_map_based_roi_component_config_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

apollo_component(
    name = "libpointcloud_map_based_roi_component.so",
    srcs = ["pointcloud_map_based_roi_component.cc"],
    hdrs = ["pointcloud_map_based_roi_component.h"],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    deps = [
        "//cyber",
        ":apollo_perception_pointcloud_map_based_roi",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/pointcloud_map_based_roi/proto:pointcloud_map_based_roi_component_config_cc_proto",
    ],
)

apollo_package()

cpplint()
