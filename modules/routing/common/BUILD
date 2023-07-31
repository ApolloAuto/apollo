load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "routing_gflags",
    srcs = ["routing_gflags.cc"],
    hdrs = ["routing_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_package()
cpplint()
