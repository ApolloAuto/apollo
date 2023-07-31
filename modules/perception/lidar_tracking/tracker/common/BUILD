load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tracked_object",
    srcs = ["tracked_object.cc"],
    hdrs = ["tracked_object.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/base:object",
        "//modules/perception/common/base:object_pool_types",
        "//modules/perception/common/base:point_cloud",
        "//modules/perception/common/algorithm/point_cloud_processing",
        "//modules/perception/common/lidar/common:feature_descriptor",
    ],
)

cc_library(
    name = "track_data",
    srcs = ["track_data.cc"],
    hdrs = ["track_data.h"],
    deps = [
        ":tracked_object",
        "//cyber",
        "//modules/perception/common/base:object_pool",
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
        "//modules/perception/common/base:object_pool",
        "@eigen",
    ],
)

apollo_package()
cpplint()
