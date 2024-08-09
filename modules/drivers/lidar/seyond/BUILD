load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_component")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

SEYOND_COPTS = ['-DMODULE_NAME=\\"seyond\\"']

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "dag/*.dag",
        "launch/*.launch",
    ]),
)

apollo_component(
    name = "libseyond_component.so",
    srcs = [
        "src/seyond_component.cpp",
        "src/seyond_driver.cpp",
    ],
    hdrs = [
        "src/seyond_component.h",
        "src/seyond_driver.h",
    ],
    copts = SEYOND_COPTS + ['-std=c++17'],
    deps = [
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/drivers/lidar/seyond/proto:seyond_config_proto",
        "//modules/drivers/lidar/seyond/proto:seyond_proto",
        "//modules/drivers/lidar/common:lidar_common",
    ],
    linkopts = [
        "-linnoclientsdk",
    ]
)

apollo_package()
cpplint()
