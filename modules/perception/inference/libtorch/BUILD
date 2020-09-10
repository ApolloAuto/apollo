load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "libtorch_net",
    srcs = ["libtorch_net.cc"],
    hdrs = ["libtorch_net.h"],
    deps = [
        "//modules/perception/inference:inference_lib",
        "//third_party:libtorch",
        "@com_google_protobuf//:protobuf",
        "@opencv//:imgcodecs",
        "@local_config_cuda//cuda:cudart",
    ],
)

cpplint()
