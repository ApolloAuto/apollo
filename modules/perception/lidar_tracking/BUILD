load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "lidar_tracking_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
        "launch/**",
    ])
)

cc_binary(
    name = "liblidar_tracking_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":lidar_tracking_component_lib"],
)

cc_library(
    name = "lidar_tracking_component_lib",
    srcs = ["lidar_tracking_component.cc"],
    hdrs = ["lidar_tracking_component.h"],
    copts = PERCEPTION_COPTS + if_profiler(),
    linkopts = ["-lleveldb"],
    deps = [
        "//cyber",
        "//modules/perception/common/onboard/inner_component_messages:lidar_inner_component_messages",
        "//modules/perception/lidar_tracking/proto:lidar_tracking_component_config_cc_proto",
        "//modules/perception/lidar_tracking/tracker/multi_lidar_fusion:mlf_engine",
        "//modules/perception/lidar_tracking/classifier/fused_classifier:fused_classifier",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
