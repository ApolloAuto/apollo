load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "lidar_output_files",
    srcs = glob([
        "conf/**",
        "dag/**",
    ])
)

cc_binary(
    name = "liblidar_output_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":lidar_output_component_lib"],
)

cc_library(
    name = "lidar_output_component_lib",
    srcs = ["lidar_output_component.cc"],
    hdrs = ["lidar_output_component.h"],
    copts = PERCEPTION_COPTS,
    linkopts = ["-lleveldb"],
    deps = [
        "//cyber",
        "//modules/perception/common/algorithm/sensor_manager",
        "//modules/perception/common/lidar/common",
        "//modules/perception/common/onboard/inner_component_messages:lidar_inner_component_messages",
        "//modules/perception/lidar_output/proto:lidar_output_component_config_cc_proto",
        "//modules/perception/common/onboard/msg_serializer:msg_serializer",
        "//modules/common_msgs/perception_msgs:perception_benchmark_cc_proto",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
