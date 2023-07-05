load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(
    default_visibility = ["//visibility:public"],
)
cc_library(
    name = "libtorch",
    deps = select({
        "//tools/platform:use_gpu": [
            "@libtorch_gpu",
        ],
        "//conditions:default": [
            "@libtorch_cpu",
        ],
    }),
)
