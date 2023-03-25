load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "libtorch_gpu",
    hdrs = glob(["include/**/*"]),
    linkopts = [
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
