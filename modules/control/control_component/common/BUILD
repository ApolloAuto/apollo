load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

apollo_cc_library(
    name = "control_gflags",
    srcs = ["control_gflags.cc"],
    hdrs = ["control_gflags.h"],
    deps = ["@com_github_gflags_gflags//:gflags"],
)

apollo_package()

cpplint()
