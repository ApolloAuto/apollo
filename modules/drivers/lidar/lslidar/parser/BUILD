load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])


install(
    name = "install",
    library_dest = "drivers/lib/lidar/lslidar/parser",
    targets = [
        ":liblslidar_convert_component.so",
    ],
)

cc_binary(
    name = "liblslidar_convert_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":lslidar_convert_component_lib"],
)

cc_library(
    name = "lslidar_convert_component_lib",
    srcs = ["lslidar_convert_component.cc"],
    hdrs = ["lslidar_convert_component.h"],
    copts = ['-DMODULE_NAME=\\"lslidar\\"'],
    deps = [
        "//cyber",
        "//modules/drivers/lidar/lslidar/parser:convert",
    ],
    alwayslink = True,
)

cc_library(
    name = "convert",
    srcs = [
	"calibration.cc",
    "convert.cc",
    "lslidar16_parser.cc",
	"lslidar32_parser.cc",
	"lslidarCXV4_parser.cc",
	"lslidarCH16_parser.cc",
	"lslidarCH32_parser.cc",
	"lslidarCH64_parser.cc",
	"lslidarCH64w_parser.cc",
	"lslidarCH120_parser.cc",
	"lslidarCH128_parser.cc",
	"lslidarCH128X1_parser.cc",
	"lslidarLS128S2_parser.cc",
    "lslidar_parser.cc",
    ],
    hdrs = [
	"calibration.h",
        "convert.h",
        "lslidar_parser.h",
    ],
    copts = ['-DMODULE_NAME=\\"lslidar\\"'],
    deps = [
        "//cyber",
        "//modules/common_msgs/sensor_msgs:pointcloud_cc_proto",
        "//modules/drivers/lidar/lslidar/proto:config_cc_proto",
        "//modules/drivers/lidar/lslidar/proto:lslidar_cc_proto",
	"@com_github_jbeder_yaml_cpp//:yaml-cpp",
    ],
)

cpplint()
