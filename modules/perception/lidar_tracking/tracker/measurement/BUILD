load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "measurement_collection",
    srcs = ["measurement_collection.cc"],
    hdrs = ["measurement_collection.h"],
    deps = [
        "//modules/perception/common/algorithm/geometry:basic",
        "//modules/perception/common/algorithm/geometry:common",
        "//modules/perception/lidar_tracking/tracker/common:tracked_object",
    ],
)

apollo_package()
cpplint()
