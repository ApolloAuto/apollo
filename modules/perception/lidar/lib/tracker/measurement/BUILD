load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "measurement_collection",
    srcs = ["measurement_collection.cc"],
    hdrs = ["measurement_collection.h"],
    deps = [
        "//modules/perception/common/geometry:basic",
        "//modules/perception/common/geometry:common",
        "//modules/perception/lidar/lib/tracker/common:tracked_object",
    ],
)

cpplint()
