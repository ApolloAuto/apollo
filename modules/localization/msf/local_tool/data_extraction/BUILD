load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "data_extraction",
    srcs = [
        "location_exporter.cc",
        "pcd_exporter.cc",
        "rosbag_reader.cc",
    ],
    hdrs = [
        "base_exporter.h",
        "location_exporter.h",
        "pcd_exporter.h",
        "rosbag_reader.h",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
        "-lboost_program_options",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/math",
        "//modules/localization/proto:gps_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/localization/proto:measure_proto",
        "@eigen//:eigen",
        "@pcl//:pcl",
        "@ros//:ros_common",
    ],
)

cc_binary(
    name = "monitor_data_exporter",
    srcs = [
        "monitor_data_exporter.cc",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
        "-lboost_program_options",
    ],
    linkstatic = 0,
    deps = [
        ":data_extraction",
        "//modules/common:log",
        "@eigen//:eigen",
        "@pcl//:pcl",
        "@ros//:ros_common",
    ],
)

cc_binary(
    name = "compare_poses",
    srcs = [
        "compare_poses.cc",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
        "-lboost_program_options",
    ],
    linkstatic = 0,
    deps = [
        ":data_extraction",
        "//modules/common:log",
        "//modules/localization/msf/local_tool/local_visualization/offline_visual:offline_local_visualizer_lib",
        "@eigen//:eigen",
    ],
)

cpplint()
