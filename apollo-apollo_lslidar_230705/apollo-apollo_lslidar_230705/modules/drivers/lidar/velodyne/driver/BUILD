load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])


install(
    name = "install",
    library_dest = "drivers/lib/lidar/velodyne/driver",
    targets = [
        ":libvelodyne_driver_component.so",
    ],
)

cc_binary(
    name = "libvelodyne_driver_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":velodyne_driver_component_lib"],
)

cc_library(
    name = "velodyne_driver_component_lib",
    srcs = ["velodyne_driver_component.cc"],
    hdrs = ["velodyne_driver_component.h"],
    copts = ['-DMODULE_NAME=\\"velodyne\\"'],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/drivers/lidar/velodyne/driver",
    ],
    alwayslink = True
)

cc_library(
    name = "driver",
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
        "//modules/drivers/lidar/common/driver_factory:driver_base",
        "//modules/drivers/lidar/proto:config_cc_proto",
        "//modules/drivers/lidar/proto:velodyne_config_cc_proto",
    ],
)

cpplint()
