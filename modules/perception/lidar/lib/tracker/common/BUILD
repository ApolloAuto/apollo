load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tracked_object",
    srcs = [
        "tracked_object.cc",
    ],
    hdrs = [
        "tracked_object.h",
    ],
    deps = [
        "//modules/perception/base:object",
        "//modules/perception/base:object_pool_types",
        "//modules/perception/base:point_cloud",
        "//modules/perception/common/point_cloud_processing:point_cloud_processing",
        "//modules/perception/lidar/common:feature_descriptor",
        "//cybertron",
    ],
)

cc_library(
    name = "track_pool_types",
    srcs = [
        "track_pool_types.cc",
    ],
    hdrs = [
        "track_pool_types.h",
    ],
    deps = [
        ":tracked_object",
        ":track_data",
        "//modules/perception/base:object_pool",
        "//cybertron",
    ],
)

cc_library(
    name = "track_data",
    srcs = [
        "track_data.cc",
    ],
    hdrs = [
        "track_data.h",
    ],
    deps = [
        ":tracked_object",
    ],
)

cpplint()
