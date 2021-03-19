load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

ROBOSENSE_COPTS = ['-DMODULE_NAME=\\"robosense\\"']

cc_binary(
    name = "librobosense_driver_component.so",
    linkshared = True,
    linkstatic = False,
    deps = ["//modules/drivers/lidar/robosense/driver:robosense_driver_component_lib"],
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
