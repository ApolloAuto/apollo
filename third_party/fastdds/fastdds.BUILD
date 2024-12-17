load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "fastdds",
    includes = [
        ".",
    ],
    hdrs = glob(["**/*"]),
    linkopts = [
        "-lfastrtps",
        "-lfastcdr",
    ],
    visibility = ["//visibility:public"],
)