load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_component")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/platform:build_defs.bzl", "if_profiler")

package(default_visibility = ["//visibility:public"])

PERCEPTION_COPTS = ['-DMODULE_NAME=\\"perception\\"']

filegroup(
    name = "lidar_output_files",
    srcs = glob([
        "conf/**",
        "dag/**",
    ])
)

apollo_component(
    name = "liblidar_output_component.so",
    srcs = ["lidar_output_component.cc"],
    hdrs = ["lidar_output_component.h"],
    copts = PERCEPTION_COPTS + if_profiler() + ["-DENABLE_PROFILER=1"],
    linkopts = ["-lleveldb"],
    deps = [
        "//cyber",
        "//modules/perception/common/algorithm:apollo_perception_common_algorithm",
        "//modules/perception/common/lidar:apollo_perception_common_lidar",
        "//modules/perception/common/onboard:apollo_perception_common_onboard",
        "//modules/perception/lidar_output/proto:lidar_output_component_config_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_benchmark_cc_proto",
        "@boost",
    ],
)

apollo_package()
cpplint()
