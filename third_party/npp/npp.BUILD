# NPP: NVIDIA 2D Image and Signal Processing Performance Primitives
# Ref https://docs.nvidia.com/cuda/npp/index.html

# TODO(infra): merge this with @local_config_cuda and split

load("@rules_cc//cc:defs.bzl", "cc_library")

package(
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "npp",
    includes = select({
        "@apollo//tools/platform:x86_mode": ["targets/x86_64-linux/include"],
        "@apollo//tools/platform:aarch64_mode": ["targets/aarch64-linux/include"],
        "//conditions:default": [],
    }),
    linkopts = [
        "-L/usr/local/cuda/lib64",
        "-lnppc",
        "-lnppial",
        "-lnppicc",
        "-lnppidei",
        "-lnppif",
        "-lnppig",
        "-lnppim",
        "-lnppist",
        "-lnppisu",
        "-lnppitc",
        "-lnpps",
    ],
)
