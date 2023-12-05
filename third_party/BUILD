load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "libtorch",
    deps = select({
        "//tools/platform:use_nvidia_gpu": [
            "@libtorch_gpu",
        ],
        "//tools/platform:use_amd_gpu": [
            "@libtorch_gpu",
        ],
        "//conditions:default": [
            "@libtorch_cpu",
        ],
    }),
)
