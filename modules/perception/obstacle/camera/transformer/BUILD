load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "transformer",
    srcs = ["flat_camera_transformer.cc"],
    hdrs = ["flat_camera_transformer.h"],
    deps = [
        "//modules/common:log",
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
