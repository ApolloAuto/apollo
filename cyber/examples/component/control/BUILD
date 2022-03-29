load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
cc_binary(
    name = "control.so",
    linkshared = True,
    linkstatic = False,
    deps = [":control_lib"],
)
cc_library(
    name = "control_lib",
    srcs = ["control.cc"],
    hdrs = ["control.h"],
    visibility = ["//visibility:private"],
    deps = [
        "//cyber",
        "//cyber/examples/component/proto:examples_cc_proto",
    ],
)
cpplint()