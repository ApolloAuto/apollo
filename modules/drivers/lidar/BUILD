load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "drivers/addition_data/lidar",
    library_dest = "drivers/lib/lidar",
    data = [
        ":runtime_data",
    ],
    targets = [
        "liblidar_driver_component.so",
    ],
    deps = [
        "//modules/drivers/lidar/hesai:install",
        "//modules/drivers/lidar/robosense:install",
        "//modules/drivers/lidar/velodyne:install",
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

cc_library(
    name = "lidar_driver_component_lib",
    srcs = ["lidar_driver_component.cc"],
    hdrs = ["lidar_driver_component.h"],
    alwayslink = True,
    deps = [
        "//cyber",
        "//modules/drivers/lidar/common/driver_factory:lidar_driver_factory",
        "//modules/drivers/lidar/proto:config_cc_proto",
    ],
)

cc_binary(
    name = "liblidar_driver_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":lidar_driver_component_lib"],
)

cpplint()
