load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ground_struct",
    hdrs = ["ground_struct.h"],
    deps = [
        "@eigen",
    ],
)

cc_library(
    name = "ground_service",
    srcs = ["ground_service.cc"],
    hdrs = ["ground_service.h"],
    deps = [
        ":ground_struct",
        "//modules/perception/common/i_lib",
        "//modules/perception/lidar/lib/scene_manager",
        "//modules/perception/lidar/lib/scene_manager/ground_service/proto:ground_service_config_cc_proto",
    ],
)

cpplint()
