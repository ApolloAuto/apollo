load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common_flags",
    srcs = ["common_flags.cc"],
    hdrs = ["common_flags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
