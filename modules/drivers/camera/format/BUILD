load("//tools:cpplint.bzl", "cpplint")

load("@local_config_cuda//cuda:build_defs.bzl", "cuda_library")

load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

cuda_library(
    name = "convert",
    srcs = ["yuv2rgb.cu"],
    hdrs = ["yuv2rgb.h"],
    deps = [
        "//cyber",
        "@local_config_cuda//cuda:cudart",
        "@local_config_cuda//cuda:cuda_headers",
    ]
)

apollo_package(enable_source=False)
cpplint()
