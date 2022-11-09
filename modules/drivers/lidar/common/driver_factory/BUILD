load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lidar_driver_factory",
    srcs = ["lidar_driver_factory.cc"],
    hdrs = ["lidar_driver_factory.h"],
    deps = [
        ":driver_base",
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/drivers/lidar/hesai/driver",
        "//modules/drivers/lidar/proto:config_cc_proto",
        "//modules/drivers/lidar/proto:lidar_parameter_cc_proto",
        # "//modules/drivers/lidar/robosense/driver",
        "//modules/drivers/lidar/velodyne/driver",
    ],
)

cc_library(
    name = "driver_base",
    hdrs = ["driver_base.h"],
    deps = [
        "//modules/common/util",
        "//modules/common/util:util_tool",
    ],
)

cpplint()
