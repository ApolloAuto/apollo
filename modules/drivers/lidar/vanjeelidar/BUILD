load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_component")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

VANJEE_COPTS = ['-DMODULE_NAME=\\"vanjeelidar\\"']

apollo_component(
    name = "libvanjeelidar_component.so",
    copts = VANJEE_COPTS,
    srcs = [
        "src/vanjeelidar_component.cpp",
    ],
    hdrs = [
        "src/vanjeelidar_component.h",
    ],
    deps = [
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/drivers/lidar/vanjeelidar/proto:vanjeelidar_config_proto",
        "//modules/drivers/lidar/vanjeelidar/proto:vanjeelidar_proto",
        "//modules/drivers/lidar/common:lidar_common",
    ],
    linkopts = [
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
