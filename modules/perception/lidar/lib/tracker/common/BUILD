load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tracked_object",
    srcs = ["tracked_object.cc"],
    hdrs = ["tracked_object.h"],
    deps = [
        "//cyber",
        "//modules/perception/base:object",
        "//modules/perception/base:object_pool_types",
        "//modules/perception/base:point_cloud",
        "//modules/perception/common/point_cloud_processing",
        "//modules/perception/lidar/common:feature_descriptor",
    ],
)

cc_library(
    name = "track_data",
    srcs = ["track_data.cc"],
    hdrs = ["track_data.h"],
    deps = [
        ":tracked_object",
        "//cyber",
        "//modules/perception/base:object_pool",
    ],
)

cc_library(
    name = "mlf_track_data_with_track_pool_types",
    srcs = [
        "mlf_track_data.cc",
        "track_pool_types.cc",
    ],
    hdrs = [
        "mlf_track_data.h",
        "track_pool_types.h",
    ],
    deps = [
        ":track_data",
        "//cyber",
        "//modules/perception/base:object_pool",
        "@eigen",
    ],
)

cpplint()
