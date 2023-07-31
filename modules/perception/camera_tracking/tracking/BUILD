load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "omt_obstacle_tracker",
    srcs = ["omt_obstacle_tracker.cc"],
    hdrs = ["omt_obstacle_tracker.h"],
    deps = [
        "//cyber",
        "//modules/perception/camera_tracking/common:similar",
        "//modules/perception/camera_tracking/base:frame_list",
        "//modules/perception/camera_tracking/base:target",
        "//modules/perception/camera_tracking/base:obstacle_reference",
        "//modules/perception/common/camera/common",
        "//modules/perception/common:util",
        # "//modules/perception/camera/lib/interface",
        # "//modules/perception/camera/lib/obstacle/tracker/common",
        # "//modules/perception/common/sensor_manager",
        "//modules/perception/camera_tracking/proto:omt_cc_proto",
        "//modules/perception/camera_tracking/interface:base_feature_extractor",
        "//modules/perception/camera_tracking/feature_extract:tracking_feat_extractor",
        "//modules/perception/camera_tracking/interface:base_obstacle_tracker",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
