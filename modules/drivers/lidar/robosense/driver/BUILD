load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

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
        "//modules/drivers/lidar/robosense:decoder_factory",
        "//cyber",
        "//modules/common/util",
        "//modules/drivers/proto:pointcloud_cc_proto",
        "//modules/drivers/lidar/proto:robosense_proto",
        "//modules/drivers/lidar/robosense/input:input",
        "//modules/drivers/lidar/common/driver_factory:driver_base",
        "//modules/drivers/lidar/proto:config_proto",
    ],
)
