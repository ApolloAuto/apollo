load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_obstacle_tracker",
    hdrs = ["base_obstacle_tracker.h"],
    deps = [
        "//modules/perception/camera_tracking/common:camera_tracking_frame",
        "//modules/perception/common/lib/interface:base_init_options",
        "//modules/perception/common/lib/registerer",
    ],
)

cc_library(
    name = "base_feature_extractor",
    hdrs = ["base_feature_extractor.h"],
    deps = [
        "//modules/perception/camera_tracking/common:camera_tracking_frame",
        "//modules/perception/common/base:camera",
        "//modules/perception/common/lib/interface:base_init_options",
        "//modules/perception/common/lib/registerer",
    ],
)

cc_library(
    name = "interface",
    deps = [
        ":base_feature_extractor",
        ":base_obstacle_tracker",
    ],
)

apollo_package()
cpplint()
