load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "core",
    hdrs = [
        "i_alloc.h",
        "i_basic.h",
        "i_blas.h",
        "i_constant.h",
        "i_rand.h",
    ],
)

cpplint()
