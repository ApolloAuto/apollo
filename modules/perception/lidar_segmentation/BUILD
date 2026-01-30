load("//tools:apollo_package.bzl", "apollo_package", "apollo_component")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "lidar_segmentation_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
        "launch/**",
    ])
)

apollo_component(
    name = "liblidar_segmentation_component.so",
    srcs = ["lidar_segmentation_component.cc"],
    hdrs = ["lidar_segmentation_component.h"],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    linkopts = ["-lleveldb"],
    deps = [
        "//cyber",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/lidar_segmentation/proto:lidar_segmentation_component_config_cc_proto",
        "//modules/perception/lidar_segmentation/segmentor/graph_segmentation:graph_segmentation",
        "//modules/perception/lidar_segmentation/segmentor/ncut_segmentation:ncut_segmentation",
        "//modules/perception/lidar_segmentation/common:lidar_segmention_common",
    ],
)

apollo_package(enable_source=False)
cpplint()
