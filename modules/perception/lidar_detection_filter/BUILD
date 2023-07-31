load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "lidar_detection_filter_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
        "launch/**",
    ])
)

cc_binary(
    name = "liblidar_detection_filter_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":lidar_detection_filter_component_lib"],
)

cc_library(
    name = "lidar_detection_filter_component_lib",
    srcs = ["lidar_detection_filter_component.cc"],
    hdrs = ["lidar_detection_filter_component.h"],
    copts = PERCEPTION_COPTS + if_profiler(),
    deps = [
        "//cyber",
        "//modules/perception/common/onboard/inner_component_messages:lidar_inner_component_messages",
        "//modules/perception/lidar_detection_filter/proto:lidar_detection_filter_component_config_cc_proto",
        "//modules/perception/lidar_detection_filter/object_filter_bank:object_filter_bank",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
