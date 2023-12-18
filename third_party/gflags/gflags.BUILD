load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "gflags",
    includes = ["."],
    hdrs = glob(["**/*"]),
    linkopts = [
        "-L/usr/local/lib/",
        "-lgflags",
    ],
)
