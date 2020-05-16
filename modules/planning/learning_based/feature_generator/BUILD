load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "feature_generator",
    hdrs = ["feature_generator.h"],
    copts = ["-DMODULE_NAME=\\\"planning\\\""],
)

cpplint()
