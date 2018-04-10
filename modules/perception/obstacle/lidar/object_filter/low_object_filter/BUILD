load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "low_object_filter",
    srcs = ["low_object_filter.cc"],
    hdrs = ["low_object_filter.h"],
    deps = [
        "//modules/common",
        "//modules/common:log",
        "//modules/perception/common",
        "//modules/perception/common:pcl_util",
        "//modules/perception/lib/base",
        "//modules/perception/obstacle/common",
        "//modules/perception/obstacle/lidar/interface",
        "//modules/perception/lib/config_manager:config_manager",
        "@eigen//:eigen",
    ],
)


cpplint()
