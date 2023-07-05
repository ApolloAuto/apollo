load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "atlas",
    includes = [
        "include",
    ],
    linkopts = [
        "-latlas",
    ],
    linkstatic = False,
    strip_include_prefix = "include",
)

cc_library(
    name = "blas",
    includes = [
        "include",
    ],
    linkopts = [
        "-lblas",
    ],
    linkstatic = False,
    strip_include_prefix = "include",
)

cc_library(
    name = "cblas",
    includes = [
        "include",
    ],
    linkopts = [
        "-lcblas",
    ],
    linkstatic = False,
    strip_include_prefix = "include",
)

cc_library(
    name = "lapack",
    includes = [
        "include",
    ],
    linkopts = [
        "-llapack",
    ],
    linkstatic = False,
    strip_include_prefix = "include",
)
