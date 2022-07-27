load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//third_party/gpus:common.bzl", "gpu_library", "if_cuda", "if_rocm")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "torch_net",
    srcs = ["torch_net.cc"],
    hdrs = ["torch_net.h"],
    deps = [
        "//modules/perception/inference:inference_lib",
        "//third_party:libtorch",
        "@com_google_protobuf//:protobuf",
        "@opencv//:imgcodecs",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

cc_library(
    name = "torch_det",
    srcs = ["torch_det.cc"],
    hdrs = ["torch_det.h"],
    deps = [
        "//modules/perception/inference:inference_lib",
        "//third_party:libtorch",
        "@com_google_protobuf//:protobuf",
        "@opencv//:imgcodecs",
    ] + if_cuda([
        "@local_config_cuda//cuda:cudart",
    ]) + if_rocm([
        "@local_config_rocm//rocm:hip",
    ]),
)

cpplint()
