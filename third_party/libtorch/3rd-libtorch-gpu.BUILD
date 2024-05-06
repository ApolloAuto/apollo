load("@rules_cc//cc:defs.bzl", "cc_library")
load("//third_party/gpus:common.bzl", "gpu_library", "if_cuda", "if_rocm")

cc_library(
    name = "libtorch_gpu",
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-lc10",
        "-ltorch",
        "-ltorch_cpu",
		    "-lnvonnxparser",
    ] + if_cuda([
        "-lc10_cuda",
        "-ltorch_cuda",
    ])+ if_rocm([
        "-lc10_hip,
        "-ltorch_hip,
    ]),
    linkstatic = False,
    deps = if_cuda([
        "@local_config_cuda//cuda:cudart",
    ])+ if_rocm([
        "@local_config_rocm//rocm:hip",
    ])+ [
        "@local_config_python//:python_headers",
        "@local_config_python//:python_lib",
        ":libtorch_gpu_headers"
    ],
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "libtorch_gpu_headers",
    hdrs = glob(["include/torch/csrc/api/include/**/*"]),
    linkstatic = False,
    deps = [
        "@local_config_cuda//cuda:cudart",
        "@local_config_python//:python_headers",
        "@local_config_python//:python_lib",
    ],
    strip_include_prefix = "include/torch/csrc/api/include",
    visibility = ["//visibility:public"],
)
