load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_ground_detector",
    hdrs = ["base_ground_detector.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/lib/registerer",
        "//modules/perception/common/lidar/common:lidar_frame",
        "//modules/perception/common/lib/interface:base_init_options",
    ],
)

apollo_package()
cpplint()
