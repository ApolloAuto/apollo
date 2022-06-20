load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "paddle_inference",
    srcs = ["paddle_inference.cc"],
    hdrs = ["paddle_inference.h"],
    includes = ["paddle/include"],
    deps = [
        "//modules/perception/inference:inference_lib",
        "@paddleinference//:paddleinference_lib",
        #"//third_party:paddleinference",
        #"@paddleinference",
        #"@paddleinference//:paddleinference",
        "@com_google_protobuf//:protobuf",
        "@opencv//:imgcodecs",
        "@local_config_cuda//cuda:cudart",
    ],
)

cpplint()
