load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "atlas",
    includes = [
        ".",
    ],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-latlas",
    ],
    linkstatic = False,
)

cc_library(
    name = "blas",
    includes = [
        ".",
    ],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-lblas",
    ],
    linkstatic = False,
)

cc_library(
    name = "cblas",
    includes = [
        ".",
    ],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-lcblas",
    ],
    linkstatic = False,
)

cc_library(
    name = "lapack",
    includes = [
        ".",
    ],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-llapack",
    ],
    linkstatic = False,
)
