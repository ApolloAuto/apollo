load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "object_builder",
    srcs = ["object_builder.cc"],
    hdrs = ["object_builder.h"],
    deps = [
        "//modules/perception/base",
        "//modules/perception/common/geometry:common",
        "//modules/perception/common/geometry:convex_hull_2d",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lib/registerer",
        "//modules/perception/lidar/common:lidar_frame",
    ],
)

cpplint()
