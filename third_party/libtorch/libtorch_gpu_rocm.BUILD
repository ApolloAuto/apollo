load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "libtorch_gpu_rocm",
    includes = [
        ".",
        "torch/csrc/api/include",
    ],
    linkopts = [
        "-L/usr/local/libtorch_gpu/lib",
        "-lc10",
        "-lc10_hip",
        "-ltorch",
        "-ltorch_cpu",
        "-ltorch_hip",
    ],
    linkstatic = False,
    deps = [
        "@local_config_rocm//rocm:hip",
        "@local_config_python//:python_headers",
        "@local_config_python//:python_lib",
    ],
)
