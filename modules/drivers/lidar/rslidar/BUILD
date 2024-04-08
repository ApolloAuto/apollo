load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_component")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

ROBOSENSE_COPTS = ['-DMODULE_NAME=\\"rslidar\\"']

apollo_component(
    name = "librslidar_component.so",
    copts = ROBOSENSE_COPTS,
    srcs = [
        "driver/rslidar_component.cpp",
    ],
    hdrs = [
        "driver/rslidar_component.h",
    ],
    deps = [
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/drivers/lidar/rslidar/proto:rslidar_config_proto",
        "//modules/drivers/lidar/rslidar/proto:rslidar_proto",
        "//modules/drivers/lidar/common:lidar_common",
    ],
    linkopts = [
        "-lpcap",
    ]
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

apollo_package()
cpplint()
