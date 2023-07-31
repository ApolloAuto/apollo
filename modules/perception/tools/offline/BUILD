load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "visualizer",
    srcs = ["libvisualizer.so"],
    hdrs = [
        "colormap.h",
        "keycode.h",
        "visualizer.h",
    ],
    alwayslink = True,
    deps = [
        ":transform_server",
        "//modules/common/util:util_tool",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/camera/common:cipv_camera",
        "//modules/perception/common/camera/common",
        "//modules/perception/common/proto:motion_service_cc_proto",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
        "@opencv//:highgui",
    ],
)

cc_binary(
    name = "libvisualizer.so",
    srcs = [
        "visualizer.cc",
        "colormap.h",
        "keycode.h",
        "visualizer.h",
    ],
    deps = [
        ":transform_server",
        "//modules/common/util:util_tool",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/camera/common:cipv_camera",
        "//modules/perception/common/camera/common",
        "//modules/perception/common/proto:motion_service_cc_proto",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
        "@opencv//:highgui",
    ],
    linkshared = True,
    linkstatic = True,
)

cc_library(
    name = "transform_server",
    srcs = ["transform_server.cc"],
    hdrs = ["transform_server.h"],
    deps = [
        "//cyber",
        "//modules/perception/common/base",
        "//modules/perception/common/camera/common",
        "//modules/common/util:util_tool",
        "@com_github_jbeder_yaml_cpp//:yaml-cpp",
        "@eigen",
    ],
)

apollo_package()
cpplint()
