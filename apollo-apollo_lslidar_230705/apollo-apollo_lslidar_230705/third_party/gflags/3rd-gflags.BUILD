load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "gflags",
    includes = ["include"],
    hdrs = glob(["include/**/*"]),
    linkopts = [
        "-lgflags",
    ],
    strip_include_prefix = "include",
)