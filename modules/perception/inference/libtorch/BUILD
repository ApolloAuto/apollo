load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "torch_test",
    srcs = ["torch_net_test.cc"],
    deps = [
        ":libtorch_net",
        "//modules/perception/base",
        "//modules/perception/camera/common",
        "//modules/perception/camera/lib/interface",
        "//modules/perception/inference:inference_factory",
        "//modules/perception/inference:inference_lib",
        "//modules/perception/inference/utils:inference_resize_lib",
        "//modules/perception/inference/utils:inference_util_lib",
        "//third_party:libtorch",
        "@local_config_tensorrt//:tensorrt",
        "@opencv//:imgcodecs",
    ],
)

cc_library(
    name = "libtorch_net",
    srcs = ["libtorch_net.cc"],
    hdrs = ["libtorch_net.h"],
    deps = [
        "//modules/perception/inference:inference_lib",
        # "//third_party:boost",
        "//third_party:libtorch",
        "@com_google_protobuf//:protobuf",
        "@opencv//:imgcodecs",
        "@local_config_cuda//cuda:cudart",
    ],
)

cpplint()
