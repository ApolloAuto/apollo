package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "osqp",
    include_prefix = "osqp",
    includes = [
        ".",
    ],
    linkopts = [
        "-L/usr/local/lib",
        "-losqp",
    ],
)
