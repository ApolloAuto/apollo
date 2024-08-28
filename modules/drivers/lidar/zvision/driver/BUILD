load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_component(
    name = "libzvision_driver_component.so",
    #linkshared = True,
    #linkstatic = False,
    deps = [":zvision_driver_component_lib"],
)

apollo_cc_library(
    name = "zvision_driver_component_lib",
    srcs = ["zvision_driver_component.cc"],
    hdrs = ["zvision_driver_component.h"],
    copts = ['-DMODULE_NAME=\\"zvision\\"'],
    deps = [
        "//cyber",
        "//modules/common/util:message_util",
        "//modules/drivers/lidar/proto:config_cc_proto",
        "//modules/drivers/lidar/zvision/driver",
    ],
)

apollo_cc_library(
    name = "driver",
    srcs = [
        "zvision_driver.cc",
        "socket_input.cc",
    ],
    hdrs = [
        "zvision_driver.h",
        "input.h",
        "socket_input.h",
    ],
    copts = ['-DMODULE_NAME=\\"zvision\\"', '-Dez6_sample_b'],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/common/util:message_util",
        "//modules/drivers/lidar/common/driver_factory:apollo_lidar_driver_base",
        "//modules/drivers/lidar/proto:config_cc_proto",
        "//modules/drivers/lidar/proto:zvision_config_cc_proto",
        #"//modules/drivers/proto:pointcloud_proto",
    ],
)

apollo_package()

cpplint()
