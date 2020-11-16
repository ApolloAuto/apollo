load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

ROBOSENSE_COPTS = ['-DMODULE_NAME=\\"robosense\\"']

cc_binary(
    name = "librobosense_driver_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":robosense_driver_component_lib"],
)

cc_library(
    name = "robosense_driver_component_lib",
    srcs = [
        "driver/robosense_driver_component.cc",
    ],
    hdrs = [
        "driver/robosense_driver_component.h",
    ],
    copts = ROBOSENSE_COPTS,
    deps = [":driver"],
)

cc_library(
    name = "driver",
    srcs = [
        "driver/driver.cpp",
        "driver/utility.cpp",
        "input/input.cpp",
    ],
    hdrs = [
        "driver/driver.h",
        "driver/utility.h",
        "input/input.h",
    ],
    copts = ROBOSENSE_COPTS,
    deps = [
        ":decoder_factory",
        "//cyber",
        "//modules/common/util",
        "//modules/drivers/proto:pointcloud_cc_proto",
        "//modules/drivers/lidar/robosense/proto:robosense_proto",
    ],
)

cc_library(
    name = "decoder_factory",
    hdrs = [
        "decoder/decoder_base.hpp",
        "decoder/decoder_factory.hpp",
    ],
    copts = ROBOSENSE_COPTS,
    deps = [
        ":decoder_128",
        ":decoder_16",
        ":decoder_32",
        ":decoder_bp",
    ],
)

cc_library(
    name = "decoder_16",
    hdrs = [
        "decoder/decoder_16.hpp",
    ],
    copts = ROBOSENSE_COPTS,
)

cc_library(
    name = "decoder_32",
    hdrs = [
        "decoder/decoder_32.hpp",
    ],
    copts = ROBOSENSE_COPTS,
)

cc_library(
    name = "decoder_128",
    hdrs = [
        "decoder/decoder_128.hpp",
    ],
    copts = ROBOSENSE_COPTS,
)

cc_library(
    name = "decoder_bp",
    hdrs = [
        "decoder/decoder_bp.hpp",
    ],
    copts = ROBOSENSE_COPTS,
)

cpplint()
