load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libvelodyne_convert_component.so",
    deps = [":velodyne_convert_component_lib"],
    linkopts = ["-shared"],
    linkstatic = False,
)

cc_library(
    name = "velodyne_convert_component_lib",
    srcs = ["velodyne_convert_component.cc"],
    hdrs = ["velodyne_convert_component.h"],
    deps = [
        "//modules/drivers/velodyne/parser:convert",
        "//framework:cybertron",
    ],
    copts = ['-DMODULE_NAME=\\"velodyne\\"']
)

cc_library(
    name = "convert",
    srcs = [
        "calibration.cc",
        "convert.cc",
        "online_calibration.cc",
        "velodyne128_parser.cc",
        "velodyne64_parser.cc",
        "velodyne32_parser.cc",
        "velodyne16_parser.cc",
        "velodyne_parser.cc",
        "util.cc",
    ],
    hdrs = [
        "calibration.h",
        "convert.h",
        "online_calibration.h",
        "const_variables.h",
        "util.h",
        "velodyne_parser.h",
    ],
    deps = [
        "//modules/drivers/velodyne/proto:velodyne_proto",
        "//modules/drivers/proto:sensor_proto",
        "//framework:cybertron",
        "@eigen",
        "@yaml_cpp//:yaml",
    ],
)

cc_binary(
    name = "libvelodyne_compensator_component.so",
    deps = [":velodyne_compensator_component_lib"],
    linkopts = ["-shared"],
    linkstatic = False,
)

cc_library(
    name = "velodyne_compensator_component_lib",
    srcs = ["velodyne_compensator_component.cc"],
    hdrs = ["velodyne_compensator_component.h"],
    deps = [
        "//modules/drivers/velodyne/parser:compensator",
        "//framework:cybertron",
    ],
    copts = ['-DMODULE_NAME=\\"velodyne\\"']
)

cc_library(
    name = "compensator",
    srcs = [
        "compensator.cc",
    ],
    hdrs = [
        "compensator.h",
        "const_variables.h",
        "util.h",
    ],
    deps = [
        "//modules/drivers/velodyne/proto:velodyne_proto",
        "//modules/drivers/proto:sensor_proto",
        "//framework:cybertron",
        "@eigen",
    ],
)

cpplint()
