load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_component(
    name = "libzvision_convert_component.so",
    #linkshared = True,
    #linkstatic = False,
    deps = [":zvision_convert_component_lib"],
)

apollo_cc_library(
    name = "zvision_convert_component_lib",
    srcs = ["zvision_convert_component.cc"],
    hdrs = ["zvision_convert_component.h"],
    copts = ['-DMODULE_NAME=\\"zvision\\"'],
    deps = [
        "//cyber",
        "//modules/drivers/lidar/zvision/parser:convert",
    ],
)

apollo_cc_library(
    name = "convert",
    srcs = [
        "convert.cc",
        "ml30_parser.cc",
        "ml30sa1_parser.cc",
        "mlx_parser.cc",
        "ez6_parser.cc",
        "parser.cc",
    ],
    hdrs = [
        "convert.h",
        "parser.h",
    ],
    copts = ['-DMODULE_NAME=\\"zvision\\"'],
    deps = [
        "//cyber",
        "//modules/drivers/lidar/proto:zvision_config_cc_proto",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/drivers/lidar/zvision/tools:tools",
        "@boost",
        "@eigen",
    ],
)

apollo_package()
cpplint()
