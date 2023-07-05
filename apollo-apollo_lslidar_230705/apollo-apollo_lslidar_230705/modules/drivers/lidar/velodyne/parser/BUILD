load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    library_dest = "drivers/lib/lidar/velodyne/parser",
    targets = [
        ":libvelodyne_convert_component.so",
    ],
)

cc_binary(
    name = "libvelodyne_convert_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":velodyne_convert_component_lib"],
)

cc_library(
    name = "velodyne_convert_component_lib",
    srcs = ["velodyne_convert_component.cc"],
    hdrs = ["velodyne_convert_component.h"],
    copts = ['-DMODULE_NAME=\\"velodyne\\"'],
    deps = [
        "//cyber",
        "//modules/drivers/lidar/velodyne/parser:convert",
    ],
    alwayslink = True
)

cc_library(
    name = "convert",
    srcs = [
        "calibration.cc",
        "convert.cc",
        "online_calibration.cc",
        "util.cc",
        "velodyne128_parser.cc",
        "velodyne16_parser.cc",
        "velodyne32_parser.cc",
        "velodyne64_parser.cc",
        "velodyne_parser.cc",
    ],
    hdrs = [
        "calibration.h",
        "const_variables.h",
        "convert.h",
        "online_calibration.h",
        "util.h",
        "velodyne_parser.h",
    ],
    copts = ['-DMODULE_NAME=\\"velodyne\\"'],
    deps = [
        "//cyber",
        "//modules/drivers/lidar/proto:velodyne_config_cc_proto",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "@boost",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@eigen",
    ],
)

cpplint()
