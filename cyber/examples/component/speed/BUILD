load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
cc_binary(
    name = "speed.so",
    linkshared = True,
    linkstatic = False,
    deps = [":speed_lib"],
)
cc_library(
    name = "speed_lib",
    srcs = ["speed.cc"],
    hdrs = ["speed.h"],
    deps = [
        "//cyber",
        "//cyber/examples/component/proto:examples_cc_proto",
    ],
)
cpplint()