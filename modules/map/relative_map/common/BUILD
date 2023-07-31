load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "relative_map_gflags",
    srcs = ["relative_map_gflags.cc"],
    hdrs = ["relative_map_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

apollo_package()
cpplint()
