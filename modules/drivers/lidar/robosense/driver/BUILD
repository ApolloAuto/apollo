load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

ROBOSENSE_COPTS = ['-DMODULE_NAME=\\"robosense\\"']

cc_library(
    name = "robosense_driver_component_lib",
    srcs = [
        "robosense_driver_component.cc",
    ],
    hdrs = [
        "robosense_driver_component.h",
    ],
    copts = ROBOSENSE_COPTS,
    deps = [
        ":driver",
        "//cyber",
    ],
)

cc_library(
    name = "driver",
    srcs = [
        "driver.cpp",
        "utility.cpp",
    ],
    hdrs = [
        "driver.h",
        "utility.h",
    ],
    copts = ROBOSENSE_COPTS,
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/drivers/lidar/common/driver_factory:driver_base",
        "//modules/drivers/lidar/proto:config_proto",
        "//modules/drivers/lidar/proto:robosense_proto",
        "//modules/drivers/lidar/robosense:decoder_factory",
        "//modules/drivers/lidar/robosense/input",
        "//modules/drivers/proto:pointcloud_cc_proto",
    ],
)

cpplint()
