load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "motion",
    srcs = ["plane_motion.cc"],
    hdrs = ["plane_motion.h"],
    copts = [
        "-Wno-deprecated",
    ],
    deps = [
        "//modules/perception/base",
    ],
)

cpplint()
