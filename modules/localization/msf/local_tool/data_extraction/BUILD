load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "data_extraction",
    srcs = [
        "rosbag_reader.cc",
        "pcd_exporter.cc",
        "location_exporter.cc"
    ],
    hdrs = [
        "rosbag_reader.h",
        "base_exporter.h",
        "pcd_exporter.h",
        "location_exporter.h"
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
        "-lboost_program_options"
    ],
    deps = [
        "//modules/localization/proto:localization_proto",
        "//modules/localization/proto:measure_proto",
        "//modules/common:log",
        "@eigen//:eigen",
        "@ros//:ros_common",
        "@pcl//:pcl"
    ],
)

cc_binary(
    name = "monitor_data_exporter",
    srcs = [
        "monitor_data_exporter.cc"
    ],
    linkstatic = 0,
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
        "-lboost_program_options"
    ],
    deps = [
        ":data_extraction",
        "@eigen//:eigen",
        "@glog//:glog",
        "@ros//:ros_common"
    ],
)

cpplint()
