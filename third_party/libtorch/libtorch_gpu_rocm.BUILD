load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "libtorch_gpu_rocm",
    hdrs = glob(["**/*"]),
    includes = [
        ".",
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
        ":libtorch_gpu_headers",
        "@local_config_python//:python_headers",
        "@local_config_python//:python_lib",
        "@local_config_rocm//rocm:hip",
    ],
)

cc_library(
    name = "libtorch_gpu_headers",
    hdrs = glob(["torch/csrc/api/include/**/*"]),
    includes = ["torch/csrc/api/include"],
    linkstatic = False,
    strip_include_prefix = "torch/csrc/api/include",
    visibility = ["//visibility:public"],
    deps = [
        "@local_config_python//:python_headers",
        "@local_config_python//:python_lib",
        "@local_config_rocm//rocm:hip",
    ],
)
