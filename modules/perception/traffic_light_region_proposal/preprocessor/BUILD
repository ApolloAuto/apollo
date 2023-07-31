load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "multi_camera_projection",
    srcs = ["multi_camera_projection.cc"],
    hdrs = ["multi_camera_projection.h"],
    deps = [
        "//modules/perception/common/camera/common:pose",
        "//cyber",
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common",
        "//modules/perception/common/algorithm/io:io_util",
        "//modules/perception/common/algorithm/sensor_manager",
    ],
)

cc_library(
    name = "tl_preprocessor",
    srcs = ["tl_preprocessor.cc"],
    hdrs = ["tl_preprocessor.h"],
    deps = [
        ":multi_camera_projection",
        "//modules/perception/common/camera/common:pose",
        "//cyber",
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common",
        "//modules/perception/common/lib/interface:base_init_options",
        "//modules/perception/traffic_light_region_proposal/interface:base_tl_preprocessor",
        "//modules/perception/common/lib/registerer",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
    alwayslink = True,
)

apollo_package()
cpplint()
