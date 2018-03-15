load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "kalman_filter_1d",
    srcs = ["kalman_filter_1d.cc"],
    hdrs = ["kalman_filter_1d.h"],
    deps = [
        "@eigen",
        "@yaml_cpp//:yaml",
    ],
)

cc_library(
    name = "filter",
    srcs = ["object_camera_filter.cc"],
    hdrs = ["object_camera_filter.h"],
    deps = [
        ":kalman_filter_1d",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/camera/common",
        "//modules/perception/obstacle/camera/interface",
        "@eigen",
        "@opencv2//:core",
        "@yaml_cpp//:yaml",
    ],
)

cpplint()
