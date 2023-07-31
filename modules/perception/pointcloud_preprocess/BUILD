load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "pointcloud_preprocess_files",
    srcs = glob([
        "conf/**",
        "dag/**",
        "data/**",
        "launch/**",
    ])
)

cc_binary(
    name = "libpointcloud_preprocess_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":pointcloud_preprocess_component_lib"],
)

cc_library(
    name = "pointcloud_preprocess_component_lib",
    srcs = ["pointcloud_preprocess_component.cc"],
    hdrs = ["pointcloud_preprocess_component.h"],
    copts = PERCEPTION_COPTS + if_profiler(),
    linkopts = ["-lleveldb"],
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/perception/common/algorithm/sensor_manager",
        "//modules/perception/common/lidar/common",
        "//modules/perception/common/onboard/transform_wrapper",
        "//modules/perception/common/onboard/inner_component_messages:lidar_inner_component_messages",
        "//modules/perception/pointcloud_preprocess/proto:pointcloud_preprocess_component_config_cc_proto",
        "//modules/perception/pointcloud_preprocess/preprocessor:pointcloud_preprocessor",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
