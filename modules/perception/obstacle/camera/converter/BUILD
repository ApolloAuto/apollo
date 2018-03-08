load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "camera_converter_geometry",
    srcs = ["geometry_camera_converter.cc"],
    hdrs = ["geometry_camera_converter.h"],
    deps = [
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/camera/common",
        "//modules/perception/obstacle/camera/interface",
        "@eigen//:eigen",
        "@opencv2//:core",
        "@yaml_cpp//:yaml",
    ],
)

cpplint()
