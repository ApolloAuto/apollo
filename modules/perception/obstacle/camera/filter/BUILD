load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "filter",
    srcs = ["object_camera_filter.cc"],
    hdrs = ["object_camera_filter.h"],
    deps = [
        "//modules/common/math:kalman_filter_1d",
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
