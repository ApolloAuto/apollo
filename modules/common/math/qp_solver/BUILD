load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "qp_solver",
    srcs = ["qp_solver.cc"],
    hdrs = ["qp_solver.h"],
    deps = [
        "@eigen",
    ],
)

cpplint()
