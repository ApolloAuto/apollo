load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "get_object",
    srcs = ["get_object.cc"],
    hdrs = ["get_object.h"],
    deps = [
        "//modules/perception/pipeline:plugin",
        "@boost",
        "@eigen",
    ],
)

cpplint()
