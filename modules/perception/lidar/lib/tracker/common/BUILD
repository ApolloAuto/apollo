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
        "//framework:cybertron",
    ],
)

cpplint()
