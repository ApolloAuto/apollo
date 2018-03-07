load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "flat_transformer",
    srcs = ["flat_camera_transformer.cc"],
    hdrs = ["flat_camera_transformer.h"],
    deps = [
        "@yaml_cpp//:yaml",
        "@eigen//:eigen",
        "@opencv2//:core",
        "//modules/perception/lib/base:base",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/obstacle/camera/common:common",
        "//modules/perception/obstacle/camera/interface:interface",
    ]
)

cpplint()
