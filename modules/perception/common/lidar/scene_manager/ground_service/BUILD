load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ground_struct",
    hdrs = ["ground_struct.h"],
    alwayslink = True,
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
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/algorithm/i_lib",
        "//modules/perception/common/lidar/scene_manager",
        "//modules/perception/common/lidar/scene_manager/ground_service/proto:ground_service_config_cc_proto",
        "//modules/perception/common:util",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
