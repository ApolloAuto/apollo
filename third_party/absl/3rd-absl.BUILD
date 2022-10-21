load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "absl",
    includes = [
        "include",
    ],
    srcs = glob(["lib/*.so*"]),
    hdrs = glob(["include/**/*"]),
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)