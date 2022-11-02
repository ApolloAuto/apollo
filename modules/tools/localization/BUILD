load("@rules_python//python:defs.bzl", "py_binary")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

py_binary(
    name = "evaluate_compare",
    srcs = ["evaluate_compare.py"],
)

install(
    name = "install",
    py_dest = "tools/localization",
    targets = [
        ":evaluate_compare",
    ]

)