load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "shared_library",
    srcs = ["shared_library.cc"],
    hdrs = ["shared_library.h"],
    linkopts = ["-ldl"],
    deps = [
        ":exceptions",
    ],
)

cc_library(
    name = "exceptions",
    hdrs = ["exceptions.h"],
)

cpplint()
