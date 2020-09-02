load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "librobosense_driver_component.so",
    linkopts = ["-shared"],
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
    copts = ['-DMODULE_NAME=\\"robosense\\"'],
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
    copts = ['-DMODULE_NAME=\\"robosense\\"'],
    deps = [
        ":decoder",
    ],
)

cc_library(
    name = "decoder",
    hdrs = [
        "decoder/decoder_128.hpp",
        "decoder/decoder_16.hpp",
        "decoder/decoder_32.hpp",
        "decoder/decoder_base.hpp",
        "decoder/decoder_bp.hpp",
        "decoder/decoder_factory.hpp",
    ],
    copts = ['-DMODULE_NAME=\\"robosense\\"'],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/drivers/proto:pointcloud_cc_proto",
        "//modules/drivers/robosense/proto:robosense_proto",
    ],
)

cpplint()
