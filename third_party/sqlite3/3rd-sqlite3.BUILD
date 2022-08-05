load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "sqlite3",
    includes = [
        "include",
    ],
    linkopts = [
        "-lsqlite3",
    ],
    linkstatic = False,
    strip_include_prefix = "include",
)
