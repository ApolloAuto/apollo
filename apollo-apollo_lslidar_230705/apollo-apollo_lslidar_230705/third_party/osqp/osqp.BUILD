load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "osqp",
    include_prefix = "osqp",
    includes = [
        ".",
    ],
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-losqp",
    ],
)
