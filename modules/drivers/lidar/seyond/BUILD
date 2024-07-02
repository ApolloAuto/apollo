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
    name = "libseyond_driver_component.so",
    srcs = [
        "driver/seyond_component.cc",
        "driver/seyond_driver.cc",
    ],
    hdrs = [
        "driver/seyond_component.h",
        "driver/seyond_driver.h",
    ],
    copts = SEYOND_COPTS + ['-std=c++17'],
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/drivers/lidar/seyond/proto:seyond_cc_proto",
        "//modules/drivers/lidar/seyond/proto:seyond_config_cc_proto",
    ],
    linkopts = [
        "-linnoclientsdk",
    ]
)

apollo_package()
cpplint()
