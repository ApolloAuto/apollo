load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

# TODO(all): move blas/cblas to deps section
cc_library(
    name = "caffe",
    includes = [
        ".",
    ],
    linkopts = [
        "-L/opt/apollo/pkgs/caffe/lib",
        "-lblas",
        "-lcblas",
        "-lz",
        "-ldl",
        "-lm",
        "-lcaffe",
    ],
    deps = [
        "@boost",
        "@opencv",
    ],
)
