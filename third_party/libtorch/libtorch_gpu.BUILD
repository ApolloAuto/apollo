load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "libtorch_gpu",
    hdrs = glob(["**/*"]),
    includes = ["."],
    linkopts = [
        "-L/usr/local/libtorch_gpu/lib",
        "-lc10",
        "-lc10_cuda",
        "-ltorch",
        "-ltorch_cpu",
        "-ltorch_cuda",
        "-lnvonnxparser",
    ],
    linkstatic = False,
    deps = [
        "@local_config_cuda//cuda:cudart",
        "@local_config_python//:python_headers",
        "@local_config_python//:python_lib",
        ":libtorch_gpu_headers"
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "libtorch_gpu_headers",
    hdrs = glob(["torch/csrc/api/include/**/*"]),
    includes = ["torch/csrc/api/include"],
    linkstatic = False,
    deps = [
        "@local_config_cuda//cuda:cudart",
        "@local_config_python//:python_headers",
        "@local_config_python//:python_lib",
    ],
    strip_include_prefix = "torch/csrc/api/include",
    visibility = ["//visibility:public"],
)
