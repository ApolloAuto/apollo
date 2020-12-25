load("//tools:cpplint.bzl", "cpplint")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

ROBOSENSE_COPTS = ['-DMODULE_NAME=\\"robosense\\"']

cc_library(
    name = "input",
    srcs = ["input.cc"],
    hdrs = ["input.h"],
    copts = ROBOSENSE_COPTS,
)

cpplint()
