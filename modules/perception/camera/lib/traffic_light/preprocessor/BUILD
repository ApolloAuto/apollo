load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "multi_camera_projection",
    srcs = ["multi_camera_projection.cc"],
    hdrs = ["multi_camera_projection.h"],
    deps = [
        ":pose",
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/common/io:io_util",
        "//modules/perception/common/sensor_manager",
    ],
)

cc_library(
    name = "pose",
    srcs = ["pose.cc"],
    hdrs = ["pose.h"],
    deps = [
        "//cyber",
        "//modules/common/util:eigen_defs",
        "//modules/perception/base",
    ],
)

cc_library(
    name = "tl_preprocessor",
    srcs = ["tl_preprocessor.cc"],
    hdrs = ["tl_preprocessor.h"],
    deps = [
        ":multi_camera_projection",
        ":pose",
        "//cyber",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/camera/lib/interface:base_init_options",
        "//modules/perception/lib/config_manager",
        "@com_github_gflags_gflags//:gflags",
        "@eigen",
    ],
)

cpplint()
