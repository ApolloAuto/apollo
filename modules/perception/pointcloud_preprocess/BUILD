load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_component", "apollo_package")
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
    ]),
)

apollo_cc_library(
    name = "apollo_perception_pointcloud_preprocess",
    srcs = [
        "preprocessor/pointcloud_preprocessor.cc",
    ],
    hdrs = [
        "interface/base_pointcloud_preprocessor.h",
        "preprocessor/pointcloud_preprocessor.h",
    ],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/perception/common:perception_common_util",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/lib:apollo_perception_common_lib",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/pointcloud_preprocess/preprocessor/proto:pointcloud_preprocessor_config_cc_proto",
        "//modules/perception/pointcloud_preprocess/proto:pointcloud_preprocess_component_config_cc_proto",
        "@eigen",
    ],
)

apollo_component(
    name = "libpointcloud_preprocess_component.so",
    srcs = ["pointcloud_preprocess_component.cc"],
    hdrs = ["pointcloud_preprocess_component.h"],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    linkopts = ["-lleveldb"],
    deps = [
        "//cyber",
        ":apollo_perception_pointcloud_preprocess",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/pointcloud_preprocess/proto:pointcloud_preprocess_component_config_cc_proto",
    ],
)

apollo_package()

cpplint()
