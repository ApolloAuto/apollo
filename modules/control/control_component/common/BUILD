load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

cc_binary(
    name = "libcontrol_gflags.so",
    linkshared = True,
    linkstatic = True,
    srcs = [
        "control_gflags.cc",
        "control_gflags.h",
    ],
    copts = CONTROL_COPTS,
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "control_gflags",
    srcs = ["libcontrol_gflags.so"],
    hdrs = ["control_gflags.h"],
    copts = CONTROL_COPTS,
    alwayslink = True,
    deps = [],
)

apollo_package()
cpplint()