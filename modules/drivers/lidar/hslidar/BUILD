load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_component", "apollo_cc_binary", "apollo_cc_test")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

HESAI_COPTS = ['-DMODULE_NAME=\\"hesai\\"']

apollo_component(
    name = "libhesai_driver_component_2.so",
    copts = HESAI_COPTS + [
        "-fopenmp",
    ],
    srcs = [
        "driver/hesai_component.cc",
    ],
    hdrs = [
#        "driver/driver.h",
        "driver/hesai_component.h",
    ],
    deps = [
        "//modules/drivers/lidar/common:lidar_common",
        "//modules/drivers/lidar/hslidar/proto:hesai_proto",
        "//modules/drivers/lidar/hslidar/proto:hesai_config_proto",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//cyber",
    ],
    linkopts = [
        "-lgomp", "-lssl", "-pthread", "-lm", "-lhesai2-driver",
    ]
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "dag/*.dag",
        "launch/*.launch",
        "correction/*/*.dat",
        "correction/*/*.csv",
    ]),
)

apollo_package()
cpplint()
