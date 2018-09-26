load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "object_builder",
    srcs = [
        "object_builder.cc",
    ],
    hdrs = [
        "object_builder.h",
    ],
    deps = [
        "//modules/perception/base:base",
        "//modules/perception/common/geometry:common",
        "//modules/perception/common/geometry:convex_hull_2d",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/lib/io:file_util",
        "//modules/perception/lib/registerer:registerer",
        "//modules/perception/lidar/common:lidar_frame",
    ],
)

cpplint()
