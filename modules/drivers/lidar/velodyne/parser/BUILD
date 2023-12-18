load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_component")

package(default_visibility = ["//visibility:public"])

apollo_component(
    name = "libvelodyne_convert_component.so",
    srcs = [
        "velodyne_convert_component.cc",
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
        "velodyne_convert_component.h",
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

apollo_package()
cpplint()
