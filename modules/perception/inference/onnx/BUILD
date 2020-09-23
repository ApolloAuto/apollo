load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "onnx_obstacle_detector",
    srcs = ["onnx_obstacle_detector.cc"],
    hdrs = [
        "onnx_obstacle_detector.h",
    ],
    deps = [
        "//modules/perception/common:perception_gflags",
        "//modules/perception/inference:inference_lib",
        "@local_config_tensorrt//:tensorrt",
        "@opencv//:core",
        "@opencv//:highgui",
        "@opencv//:imgcodecs",
        "@opencv//:imgproc",
    ],
)

cc_library(
    name = "libtorch_obstacle_detector",
    srcs = ["libtorch_obstacle_detector.cc"],
    hdrs = [
        "libtorch_obstacle_detector.h",
    ],
    deps = [
        "//cyber/common:log",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/inference:inference_lib",
        "//third_party:libtorch",
    ],
)

cc_test(
    name = "libtorch_obstacle_detector_test",
    size = "small",
    srcs = ["libtorch_obstacle_detector_test.cc"],
    deps = [
        ":libtorch_obstacle_detector",
        "//modules/perception/common:perception_gflags",
        "//third_party:libtorch",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
