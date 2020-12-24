load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lidar_driver_factory",
    srcs = ["lidar_driver_factory.cc"],
    hdrs = ["lidar_driver_factory.h"],
    deps = [
        "//cyber",
        "//modules/common/util:factory",
        ":driver_base",
        "//modules/drivers/lidar/proto:lidar_parameter",
        "//modules/drivers/lidar/proto:config",
        "//modules/drivers/lidar/hesai/driver:driver",
        "//modules/drivers/lidar/robosense/driver:driver",
        "//modules/drivers/lidar/velodyne/driver:driver",
    ],
)

cc_library(
    name = "driver_base",
    hdrs = ["driver_base.h"],
    deps = [
        "//modules/common/util",
        "//modules/common/util:factory",
    ],
)

cpplint()
