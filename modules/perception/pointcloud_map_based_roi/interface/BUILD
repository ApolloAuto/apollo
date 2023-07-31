load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_roi_filter",
    hdrs = ["base_roi_filter.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/lidar/common:lidar_frame",
    ],
)

apollo_package()
cpplint()
