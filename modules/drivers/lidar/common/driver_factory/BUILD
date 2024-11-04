load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "lidar_driver_factory",
    srcs = ["lidar_driver_factory.cc"],
    hdrs = ["lidar_driver_factory.h",],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/common/util:common_util",
        "//modules/drivers/lidar/proto:config_cc_proto",
        "//modules/drivers/lidar/proto:lidar_parameter_cc_proto",
        "//modules/drivers/lidar/velodyne/driver:velodyne_driver_lib",
    ],
)

apollo_cc_library(
    name = "apollo_lidar_driver_base",
    hdrs = ["driver_base.h"],
    deps = [
        "//modules/common/util:common_util",
        "//modules/common/util:util_tool",
    ],
)

apollo_package()
cpplint()
