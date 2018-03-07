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
        "//modules/perception/obstacle/base",
        "@eigen",
        "@undistort",
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
        "//modules/perception/lib/pcl_util",
        "//modules/perception/obstacle/base",
        "@eigen",
        "@undistort",
    ],
)

cc_library(
    name = "caffe_bridge",
    hdrs = [
        "caffe_bridge.hpp",
    ],
)

# cc_library(
#     name = "cnn_adapter",
#     srcs = [
#         "cnn_adapter.cc",
#     ],
#     hdrs = [
#         "cnn_adapter.h",
#     ],
#     deps = [
#         ":caffe_bridge",
#         "//modules/perception/lib/pcl_util",
#         "@eigen",
#         "@undistort",
#     ],
# )

cpplint()
