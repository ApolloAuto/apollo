load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "spatio_temporal_ground_detector",
    srcs = [
        "spatio_temporal_ground_detector.cc",
    ],
    hdrs = [
        "spatio_temporal_ground_detector.h",
    ],
    deps = [
        "@eigen",
        "//framework:cybertron",
        "//modules/perception/common/i_lib/pc:i_ground",
        "//modules/perception/common/point_cloud_processing",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/io:file_util",
        "//modules/perception/lib/io:protobuf_util",
        "//modules/perception/lidar/common:lidar_log",
        "//modules/perception/lidar/common:lidar_point_label",
        "//modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector/proto:spatio_temporal_ground_detector_config_proto",
        "//modules/perception/lidar/lib/interface:base_ground_detector",
        "//modules/perception/lidar/lib/scene_manager",
        "//modules/perception/lidar/lib/scene_manager/ground_service",
    ],
)

cpplint()
