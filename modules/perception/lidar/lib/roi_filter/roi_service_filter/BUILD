load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "roi_service_filter",
    srcs = [
        "roi_service_filter.cc",
    ],
    hdrs = [
        "roi_service_filter.h",
    ],
    deps = [
        "@eigen",
        "//framework:cybertron",
        "//modules/perception/base:base_type",
        "//modules/perception/base:blob",
        "//modules/perception/base:box",
        "//modules/perception/base:common",
        "//modules/perception/base:frame",
        "//modules/perception/base:image",
        "//modules/perception/base:impending_collision_edge",
        "//modules/perception/base:object",
        "//modules/perception/base:object_pool_types",
        "//modules/perception/base:point_cloud",
        "//modules/perception/base:syncedmem",
        "//modules/perception/lidar/common:lidar_frame",
        "//modules/perception/lidar/common:lidar_log",
        "//modules/perception/lidar/common:lidar_point_label",
        "//modules/perception/lidar/lib/interface:base_roi_filter",
        "//modules/perception/lidar/lib/scene_manager",
        "//modules/perception/lidar/lib/scene_manager/roi_service",
    ],
)

cpplint()
