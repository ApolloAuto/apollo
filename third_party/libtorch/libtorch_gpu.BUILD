load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "libtorch_gpu",
    hdrs = glob(["**/*"]),
    includes = ["."],
    linkopts = select({
        "@local_config_rocm//rocm:using_hipcc": [
            "-L/usr/local/libtorch_gpu/lib",
            "-lc10",
            "-lc10_hip",
            "-ltorch",
            "-ltorch_cpu",
            "-ltorch_hip",
        ],
        "//conditions:default": [
            "-L/usr/local/libtorch_gpu/lib",
            "-lc10",
            "-lc10_cuda",
            "-ltorch",
            "-ltorch_cpu",
            "-ltorch_cuda",
            "-lnvonnxparser",
        ],
    }),
    linkstatic = False,
    visibility = ["//visibility:public"],
    deps = select({
        "@local_config_rocm//rocm:using_hipcc": [
            ":libtorch_gpu_headers",
            "@local_config_python//:python_headers",
            "@local_config_python//:python_lib",
            "@local_config_rocm//rocm:hip",
        ],
        "//conditions:default": [
            ":libtorch_gpu_headers",
            "@local_config_cuda//cuda:cudart",
            "@local_config_python//:python_headers",
            "@local_config_python//:python_lib",
        ],
    }),
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
    ] + select({
        "@local_config_rocm//rocm:using_hipcc": [
            "@local_config_rocm//rocm:hip",
        ],
        "//conditions:default": [
            "@local_config_cuda//cuda:cudart",
        ],
    }),
)
