load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
cc_binary(
    name = "distance.so",
    linkshared = True,
    linkstatic = False,
    deps = [":distance_lib"],
)
cc_library(
    name = "distance_lib",
    srcs = ["distance.cc"],
    hdrs = ["distance.h"],
    deps = [
        "//cyber",
        "//cyber/examples/component/proto:examples_cc_proto",
    ],
)
cpplint()