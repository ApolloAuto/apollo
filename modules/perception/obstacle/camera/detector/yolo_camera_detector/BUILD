load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

# cc_library(
#     name = "region_output",
#     srcs = [
#         "region_output.cc",
#     ],
#     hdrs = [
#         "region_output.h",
#     ],
# )
#
# cc_library(
#     name = "yolo_camera_detector",
#     srcs = [
#         "yolo_camera_detector.cc",
#     ],
#     hdrs = [
#         "yolo_camera_detector.h",
#     ],
# )

cc_library(
    name = "util",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    deps = [
        "//modules/common:log",
        "//modules/perception/obstacle/base:perception_obstacle_base",
        "@caffe//:lib",
        "@eigen//:eigen",
    ],
)

cpplint()
