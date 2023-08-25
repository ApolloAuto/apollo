load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "perception_visualizer",
    srcs = ["visualizer.cc"],
    hdrs = [
        "colormap.h",
        "keycode.h",
        "visualizer.h",
    ],
    deps = [
        ":perception_transform_server",
        "//modules/common/util:util_tool",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "//modules/perception/common/proto:motion_service_cc_proto",
        "@opencv//:highgui",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)

apollo_cc_library(
    name = "perception_transform_server",
    srcs = ["transform_server.cc"],
    hdrs = ["transform_server.h"],
    deps = [
        "//cyber",
        "//modules/common/util:util_tool",
        "//modules/perception/common/base:apollo_perception_common_base",
        "//modules/perception/common/camera:apollo_perception_common_camera",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@eigen",
    ],
)

apollo_package()

cpplint()
