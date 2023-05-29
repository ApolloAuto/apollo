load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    library_dest = "drivers/lib/lidar/lslidar/compensator",
    targets = [
        ":liblslidar_compensator_component.so",
    ],
)

cc_binary(
    name = "liblslidar_compensator_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":compensator_component_lib"],
)

cc_library(
    name = "compensator_component_lib",
    srcs = ["compensator_component.cc"],
    hdrs = ["compensator_component.h"],
    copts = ['-DMODULE_NAME=\\"lslidar\\"'],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder",
        "//modules/drivers/lidar/lslidar/compensator:compensator_lib",
        "//modules/drivers/lidar/lslidar/proto:lslidar_cc_proto",
    ],
        alwayslink = True,
)



cc_library(
    name = "compensator_lib",
    srcs = ["compensator.cc"],
    hdrs = ["compensator.h"],
    copts = ['-DMODULE_NAME=\\"lslidar\\"'],
    deps = [
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/drivers/lidar/lslidar/proto:config_cc_proto",
        "//modules/transform:buffer",
        "@eigen",
    ],
)

cpplint()
