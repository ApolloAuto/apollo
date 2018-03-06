load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "kalman_filter_1d",
    srcs = ["kalman_filter_1d.cc"],
    hdrs = ["kalman_filter_1d.h"],
    deps = [
        "@yaml_cpp//:yaml",
        "@eigen//:eigen",
    ]
)

cc_library(
    name = "object_camera_filter",
    srcs = ["object_camera_filter.cc"],
    hdrs = ["object_camera_filter.h"],
    deps = [
        "@yaml_cpp//:yaml",
        "@eigen//:eigen",
        "@opencv2//:core",
        "//modules/perception/lib/base:base",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/obstacle/camera/common:common",
        "//modules/perception/obstacle/camera/interface:interface",
        ":kalman_filter_1d"
    ]
)

cpplint()
