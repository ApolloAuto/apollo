load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "libtorch",
    deps = select({
        "//tools/platform:use_nvidia_gpu": [
            "@libtorch_gpu_cuda",
        ],
        "//tools/platform:use_amd_gpu": [
            "@libtorch_gpu_rocm",
        ],
        "//conditions:default": [
            "@libtorch_cpu",
        ],
    }),
)
