load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
cc_binary(
    name = "cal2.so",
    linkshared = True,
    linkstatic = False,
    deps = [":cal2_lib"],
)
cc_library(
    name = "cal2_lib",
    srcs = ["cal2.cc"],
    hdrs = ["cal2.h"],
    visibility = ["//visibility:private"],
    deps = [
        "//cyber",
        "//cyber/examples/component/proto:examples_cc_proto",
    ],
)
cpplint()