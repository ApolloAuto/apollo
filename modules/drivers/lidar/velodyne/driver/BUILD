load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_component")

package(default_visibility = ["//visibility:public"])

apollo_component(
    name = "libvelodyne_driver_component.so",
    hdrs = ["velodyne_driver_component.h",],
    srcs = ["velodyne_driver_component.cc",],
    deps = [":velodyne_driver_lib"],
)

apollo_cc_library(
    name = "velodyne_driver_lib",
    srcs = [
        "driver.cc",
        "driver64.cc",
        "input.cc",
        "socket_input.cc",
    ],
    hdrs = [
        "driver.h",
        "input.h",
        "socket_input.h",
    ],
    copts = ['-DMODULE_NAME=\\"velodyne\\"'],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/drivers/lidar/common/driver_factory:apollo_lidar_driver_base",
        "//modules/drivers/lidar/proto:config_cc_proto",
        "//modules/drivers/lidar/proto:velodyne_config_cc_proto",
    ],
)

apollo_package()
cpplint()
