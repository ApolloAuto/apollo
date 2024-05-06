load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "osqp",
    include_prefix = "osqp",
    includes = [
        ".",
    ],
    hdrs = glob(["**/*"]),
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-losqp",
    ],
)
