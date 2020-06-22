load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "roi_service",
    srcs = ["roi_service.cc"],
    hdrs = ["roi_service.h"],
    deps = [
        "//modules/common/util",
        "//modules/perception/lib/config_manager",
        "//modules/perception/lidar/lib/scene_manager:scene_service",
        "//modules/perception/lidar/lib/scene_manager/roi_service/proto:roi_service_cc_proto",
        "@eigen",
    ],
)

cpplint()
