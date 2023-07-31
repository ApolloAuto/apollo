load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "libtorch_test_files",
    srcs = glob([
        "test_data/**",
    ])
)

cc_library(
    name = "torch_net",
    srcs = ["torch_net.cc"],
    hdrs = ["torch_net.h"],
    alwayslink = True,
    deps = [
        "//modules/perception/common/inference:inference_lib",
        "@libtorch_gpu",
        "@com_google_protobuf//:protobuf",
        "@opencv//:imgcodecs",
        "@local_config_cuda//cuda:cudart",
    ],
)

apollo_package()
cpplint()
