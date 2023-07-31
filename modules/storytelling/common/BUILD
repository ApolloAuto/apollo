load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "storytelling_gflags",
    srcs = ["storytelling_gflags.cc"],
    hdrs = ["storytelling_gflags.h"],
    copts = [
        "-DMODULE_NAME=\\\"storytelling\\\"",
    ],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_package()
cpplint()
