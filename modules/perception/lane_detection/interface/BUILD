load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "base_lane_postprocessor",
    hdrs = ["base_lane_postprocessor.h"],
    deps = [
        "//modules/perception/common/lib/interface:base_init_options",
        "//modules/perception/common/camera/common",
        "//modules/perception/common/lib/registerer",
    ],
)

cc_library(
    name = "base_lane_detector",
    hdrs = ["base_lane_detector.h"],
    deps = [
        "//modules/perception/common/lib/interface:base_init_options",
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common",
        "//modules/perception/common/lib/registerer",
    ],
)

cc_library(
    name = "base_camera_perception",
    hdrs = ["base_camera_perception.h"],
    deps = [
        "//modules/perception/common/lib/interface:base_init_options",
        "//modules/perception/common/camera/common",
        "//modules/perception/common/lib/registerer",
    ],
)

cc_library(
    name = "interface",
    deps = [
        ":base_lane_detector",
        ":base_lane_postprocessor",
        ":base_camera_perception",
    ],
)

apollo_package()
cpplint()
