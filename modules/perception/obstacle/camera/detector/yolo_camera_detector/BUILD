load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "util",
    srcs = ["util.cc"],
    hdrs = ["util.h"],
    deps = [
        "//modules/common:log",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/camera/common",
        "@caffe//:lib",
        "@eigen",
        "@opencv2//:core",
    ],
)

cc_library(
    name = "yolo_camera_detector",
    srcs = ["yolo_camera_detector.cc"],
    hdrs = ["yolo_camera_detector.h"],
    deps = [
        ":util",
        "//modules/common:log",
        "//modules/perception/common",
        "//modules/perception/cuda_util",
        "//modules/perception/lib/base",
        "//modules/perception/lib/config_manager",
        "//modules/perception/obstacle/base",
        "//modules/perception/obstacle/camera/common:cnn_adapter",
        "//modules/perception/obstacle/camera/detector/common:feature_extractor",
        "//modules/perception/obstacle/camera/detector/common/proto:tracking_feature_proto",
        "//modules/perception/obstacle/camera/detector/yolo_camera_detector/proto:yolo_camera_proto",
        "//modules/perception/obstacle/camera/dummy",
        "//modules/perception/obstacle/camera/interface",
        "//modules/perception/proto:yolo_camera_detector_config_lib_proto",
        "@caffe//:lib",
        "@eigen",
    ],
)

cc_test(
    name = "yolo_camera_detector_test",
    size = "small",
    srcs = [
        "yolo_camera_detector_test.cc",
    ],
    data = [
        "//modules/perception:perception_data",
        "//modules/perception:perception_model",
        "//modules/perception/conf:perception_config",
    ],
    deps = [
        ":yolo_camera_detector",
        "//modules/common/util",
        "//modules/perception/cuda_util",
        "@gtest//:main",
    ],
)

cpplint()
