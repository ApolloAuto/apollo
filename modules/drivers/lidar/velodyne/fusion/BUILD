load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    library_dest = "drivers/lib/lidar/velodyne/fusion",
    targets = [
        ":libvelodyne_fusion_component.so",
    ],
)

cc_binary(
    name = "libvelodyne_fusion_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":fusion_component_lib"],
)

cc_library(
    name = "fusion_component_lib",
    srcs = ["pri_sec_fusion_component.cc"],
    hdrs = ["pri_sec_fusion_component.h"],
    copts = ['-DMODULE_NAME=\\"velodyne\\"'],
    deps = [
        "//cyber",
        "//modules/drivers/lidar/proto:velodyne_config_cc_proto",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/transform:buffer",
        "@eigen",
    ],
    alwayslink = True,
)

cpplint()
