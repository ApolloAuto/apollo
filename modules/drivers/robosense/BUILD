load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "librobosense_driver_component.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":driver"],
)

cc_library(
    name = "driver",
    srcs = [
        "driver/robosense_driver_component.cc",
        "driver/driver.cpp",
        "driver/utility.cpp",
        "input/input.cpp",
    ],
    hdrs = [
        "decoder/decoder_16.hpp",
        "decoder/decoder_128.hpp",
        "decoder/decoder_32.hpp",
        "decoder/decoder_bp.hpp",
        "decoder/decoder_base.hpp",
        "decoder/decoder_factory.hpp",
        "driver/driver.h",
        "driver/utility.h",
        "input/input.h",
        "driver/robosense_driver_component.h",

    ],
    copts = ['-DMODULE_NAME=\\"robosense\\"'],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/drivers/robosense/proto:robosense_proto",
        "//modules/drivers/proto:sensor_proto",
    ],
)

cpplint()
