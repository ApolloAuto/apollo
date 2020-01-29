load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libvelodyne_convert_component.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":velodyne_convert_component_lib"],
)

cc_library(
    name = "velodyne_convert_component_lib",
    srcs = ["velodyne_convert_component.cc"],
    hdrs = ["velodyne_convert_component.h"],
    copts = ['-DMODULE_NAME=\\"velodyne\\"'],
    deps = [
        "//cyber",
        "//modules/drivers/velodyne/parser:convert",
    ],
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
        "//modules/drivers/proto:sensor_proto",
        "//modules/drivers/velodyne/proto:velodyne_proto",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@eigen",
    ],
)

cpplint()
