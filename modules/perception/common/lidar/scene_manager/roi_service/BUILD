load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "roi_service",
    srcs = ["roi_service.cc"],
    hdrs = ["roi_service.h"],
    alwayslink = True,
    deps = [
        "//modules/common/util",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/lidar/scene_manager:scene_service",
        "//modules/perception/common/lidar/scene_manager/roi_service/proto:roi_service_cc_proto",
        "//modules/perception/common:util",
        "@eigen",
    ],
)

apollo_package()
cpplint()
