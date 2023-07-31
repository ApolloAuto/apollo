load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "object_builder",
    srcs = ["object_builder.cc"],
    hdrs = ["object_builder.h"],
    deps = [
        "//modules/perception/common/base",
        "//modules/perception/common/algorithm/geometry:common",
        "//modules/perception/common/algorithm/geometry:convex_hull_2d",
        "//modules/perception/common/lib/registerer",
        "//modules/perception/common/lidar/common:lidar_frame",
    ],
)

apollo_package()
cpplint()
