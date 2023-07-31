load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "msg_adapter_gflags",
    srcs = ["msg_adapter_gflags.cc"],
    hdrs = ["msg_adapter_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ]
)

apollo_package()
cpplint()
