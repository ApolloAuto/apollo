load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
cc_binary(
    name = "cal1.so",
    linkshared = True,
    linkstatic = False,
    deps = [":cal1_lib"],
)
cc_library(
    name = "cal1_lib",
    srcs = ["cal1.cc"],
    hdrs = ["cal1.h"],
    visibility = ["//visibility:private"],
    deps = [
        "//cyber",
        "//cyber/examples/component/proto:examples_cc_proto",
    ],
)
cpplint()