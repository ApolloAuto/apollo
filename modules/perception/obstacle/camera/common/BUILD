load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    srcs = [
    ],
    hdrs = [
        "camera.h",
        "lane_object.h",
        "visual_object.h",
    ],
    deps = [
        "//modules/perception/cuda_util",
        "//modules/perception/obstacle/base",
        "@eigen",
    ],
)

cc_library(
    name = "util",
    srcs = [
        "util.cc",
    ],
    hdrs = [
        "util.h",
        "visual_object.h",
    ],
    deps = [
        "//modules/perception/common:pcl_util",
        "//modules/perception/cuda_util",
        "//modules/perception/obstacle/base",
        "@eigen",
    ],
)

cc_library(
    name = "projector",
    srcs = ["projector.cc"],
    hdrs = ["projector.h"],
    deps = [
        "//modules/common:log",
        "@eigen",
    ],
)

cc_library(
    name = "cnn_adapter",
    srcs = ["cnn_adapter.cc"],
    hdrs = ["cnn_adapter.h"],
    deps = [
        "//modules/perception/cuda_util",
        "//modules/perception/obstacle/camera/common:util",
    ],
)

cpplint()
