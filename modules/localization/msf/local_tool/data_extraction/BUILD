load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "data_extraction",
    srcs = [
        "location_exporter.cc",
        "pcd_exporter.cc",
        "cyber_record_reader.cc",
    ],
    hdrs = [
        "location_exporter.h",
        "pcd_exporter.h",
        "cyber_record_reader.h"
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
        "-lboost_program_options",
    ],
    deps = [
        "//cybertron",
        "//modules/common/math",
        "//modules/localization/proto:gps_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/localization/proto:measure_proto",
        "//modules/drivers/proto:sensor_proto",
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "@eigen//:eigen",
        "@pcl//:pcl",
    ],
)

cc_binary(
    name = "cyber_record_parser",
    srcs = [
        "cyber_record_parser.cc",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
        "-lboost_program_options",
    ],
    linkstatic = 0,
    deps = [
        ":data_extraction",
        "//cybertron",
        "@eigen//:eigen",
        "@pcl//:pcl",
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
        "//cybertron",
        "@eigen//:eigen",
    ],
)

cpplint()
