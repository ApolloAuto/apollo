load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_plugin")
load("//tools/platform:build_defs.bzl", "if_aarch64", "if_gpu")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_files",
    srcs = glob([
        "conf/**",
        "data/**",
    ]),
)

apollo_plugin(
    name = "liblidar_pointcloud_plugin.so",
    srcs = ["lidar_pointcloud.cc"],
    hdrs = ["lidar_pointcloud.h"],
    description = ":plugins.xml",
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:pointcloud_proto",
        "//cyber/ros_bridge:converter_base",
        "//cyber/proto:simple_proto",
        "//cyber/ros_bridge/proto:converter_conf_proto",
        "@ros"
    ],
)

apollo_package()

cpplint()
