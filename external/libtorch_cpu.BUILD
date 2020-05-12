package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "libtorch_cpu",
    includes = [
        "."
    ],
    linkstatic = False,
    linkopts = [
        "-L/usr/local/libtorch_cpu/lib",
        "-ltorch",
    ],
)
