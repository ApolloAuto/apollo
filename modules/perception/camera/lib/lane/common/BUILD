load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common_functions",
    srcs = ["common_functions.cc"],
    hdrs = ["common_functions.h"],
    deps = [
        "//cyber",
        "//modules/perception/base",
        "@eigen",
    ],
)

cpplint()
