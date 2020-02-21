load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "data_extraction",
    srcs = [
        "cyber_record_reader.cc",
        "location_exporter.cc",
        "pcd_exporter.cc",
    ],
    hdrs = [
        "cyber_record_reader.h",
        "location_exporter.h",
        "pcd_exporter.h",
    ],
    deps = [
        "//cyber",
        "//modules/common/math",
        "//modules/drivers/proto:sensor_proto",
        "//modules/localization/msf/common/io:localization_msf_common_io",
        "//modules/localization/proto:gps_proto",
        "//modules/localization/proto:localization_proto",
        "//modules/localization/proto:measure_proto",
        "//third_party:boost",
        "@eigen",
        "@pcl",
    ],
)

cc_binary(
    name = "cyber_record_parser",
    srcs = ["cyber_record_parser.cc"],
    linkstatic = 0,
    deps = [
        ":data_extraction",
        "//cyber",
        "//third_party:boost",
        "@eigen",
        "@pcl",
    ],
)

cc_binary(
    name = "compare_poses",
    srcs = ["compare_poses.cc"],
    deps = [
        ":data_extraction",
        "//cyber",
        "//third_party:boost",
        "@eigen",
    ],
)

cpplint()
