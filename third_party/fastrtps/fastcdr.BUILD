load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "fastcdr",
    includes = [
        ".",
    ],
    hdrs = glob(["**/*"]),
    linkopts = [
        "-L/usr/local/fast-rtps/lib",
        "-lfastcdr",
    ],
    visibility = ["//visibility:public"],
)
