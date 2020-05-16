load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "model_inference",
    hdrs = ["model_inference.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
)

cc_library(
    name = "trajectory_conv_rnn_inference",
    hdrs = ["trajectory_conv_rnn_inference.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
    deps = [
        ":model_inference",
    ],
)

cpplint()
