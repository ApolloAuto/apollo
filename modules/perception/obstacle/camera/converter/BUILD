load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "camera_converter_geometry",
    srcs = ["geometry_camera_converter.cc"],
    hdrs = ["geometry_camera_converter.h"],
    deps = [
        "@yaml_cpp//:yaml",
        "@eigen//:eigen",
        "@opencv2//:core",
        "//modules/perception/lib/base:base",
        "//modules/perception/lib/config_manager:config_manager",
        "//modules/perception/obstacle/camera/common:obstacle_camera_common",
        "//modules/perception/obstacle/camera/interface:obstacle_camera_interface"
    ]
)

cpplint()
