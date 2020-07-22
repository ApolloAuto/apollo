load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

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

# systemlibs
# TODO(infra): all the system libs should have its own section

# atlas-cblas
cc_library(
    name = "cblas",
    linkopts = [
        "-lcblas",
    ],
)
