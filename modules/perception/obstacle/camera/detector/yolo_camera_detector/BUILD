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

filegroup(
    name = "testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

# cc_test(
#     name = "yolo_camera_detector_test",
#     size = "small",
#     srcs = [
#         "yolo_camera_detector_test.cc",
#     ],
#     data = [
#         ":testdata"
#     ],
#     deps = [
#         ":yolo_camera_detector",
#         "@gtest//:main",
#     ],
# )

# cc_test(
#     name = "yolo_camera_tensorrt_detector_test",
#     size = "small",
#     srcs = [
#         "yolo_camera_tensorrt_detector_test.cc",
#     ],
#     data = [
#         ":testdata"
#     ],
#     deps = [
#         ":yolo_camera_detector",
#         "@gtest//:main",
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
