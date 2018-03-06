load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "region_output",
    srcs = ["region_output.cc"],
    hdrs = ["region_output.h"],
    deps = [
        "//modules/common:log",
        "//modules/perception/obstacle/camera/common:util",
        "//modules/perception/obstacle/camera/detector/common:util",
        "@caffe//:lib",
        "@opencv2//:core",
    ],
)

# cc_library(
#     name = "yolo_camera_detector",
#     srcs = [
#         "yolo_camera_detector.cc",
#     ],
#     hdrs = [
#         "yolo_camera_detector.h",
#     ],
#     deps = [
#         "//modules/common:log",
#         "//modules/perception/obstacle/base",
#         "@caffe//:lib",
#         "@eigen",
#     ],
# )

cc_library(
    name = "util",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    deps = [
        "//modules/common:log",
        "//modules/perception/obstacle/base",
        "@caffe//:lib",
        "@eigen",
    ],
)

cpplint()
