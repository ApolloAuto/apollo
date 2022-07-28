load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "libtorch_cpu",
    includes = [
        "include",
        "torch/csrc/api/include",
    ],
    linkopts = [
        "-lc10",
        "-ltorch",
        "-ltorch_cpu",
    ],
    linkstatic = False,
    deps = [
        "@local_config_python//:python_headers",
        "@local_config_python//:python_lib",
    ],
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)