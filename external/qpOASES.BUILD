load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "qpOASES",
    includes = [
        ".",
    ],
    linkopts = [
        "-L/usr/local/lib",
        "-lqpOASES",
    ],
    linkstatic = False,
)
