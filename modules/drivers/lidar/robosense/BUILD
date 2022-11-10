load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data = [
        ":runtime_data",
    ],
    data_dest = "drivers/addition_data/lidar/robosense",
    library_dest = "drivers/lib/lidar/robosense",
    targets = [
        "libsuteng_driver_component.so",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "conf/*.txt",
        "conf/*.conf",
        "dag/*.dag",
        "launch/*.launch",
        "params/**",
    ]),
)

cc_binary(
    name = "libsuteng_driver_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":driver"],
)

cc_library(
    name = "driver",
    srcs = glob(
        ["**/*.cpp"],
    ),
    hdrs = glob(
        ["**/*.h"],
    ),
    copts = ['-DMODULE_NAME=\\"suteng\\"'],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/drivers/lidar/robosense/proto:sensor_suteng_cc_proto",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@eigen",
        "@local_config_pcl//:pcl",
    ],
    alwayslink = True,
)

cpplint()
