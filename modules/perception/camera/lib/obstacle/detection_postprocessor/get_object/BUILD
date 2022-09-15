load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "get_object",
    srcs = ["get_object.cc"],
    hdrs = ["get_object.h"],
    deps = [
        "//modules/perception/pipeline:plugin",
        "@boost",
        "@eigen",
        "@com_google_absl//:absl",
        "@local_config_cuda//cuda:cudart",
        "@local_config_cuda//cuda:cuda_headers",
    ],
)

cpplint()
