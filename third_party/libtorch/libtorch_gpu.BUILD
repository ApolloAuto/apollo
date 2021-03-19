load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "libtorch_gpu",
    includes = [
        ".",
        "torch/csrc/api/include",
    ],
    linkopts = [
        "-L/usr/local/libtorch_gpu/lib",
        "-lc10",
        "-lc10_cuda",
        "-ltorch",
        "-ltorch_cpu",
        "-ltorch_cuda",
    ],
    linkstatic = False,
    deps = [
        "@local_config_cuda//cuda:cudart",
        "@local_config_python//:python_headers",
        "@local_config_python//:python_lib",
    ],
)
