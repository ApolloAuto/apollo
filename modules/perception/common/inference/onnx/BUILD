load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "onnx_test_files",
    srcs = glob([
        "testdata/**",
    ])
)

cc_library(
    name = "onnx_obstacle_detector",
    srcs = ["onnx_obstacle_detector.cc"],
    alwayslink = True,
    hdrs = [
        "onnx_obstacle_detector.h",
    ],
    deps = [
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/inference:inference_lib",
        "@local_config_cuda//cuda:cudart",
        "@local_config_tensorrt//:tensorrt",
    ],
)

cc_library(
    name = "libtorch_obstacle_detector",
    srcs = ["libtorch_obstacle_detector.cc"],
    alwayslink = True,
    hdrs = [
        "libtorch_obstacle_detector.h",
    ],
    deps = [
        "//cyber",
        "//modules/perception/common:perception_gflags",
        "//modules/perception/common/inference:inference_lib",
        "@libtorch_gpu",
    ],
)

apollo_package()
cpplint()
