load("@rules_python//python:defs.bzl", "py_binary")
load("//tools/install:install.bzl", "install_files")

package(
    default_visibility = ["//visibility:public"],
)

# FIXME(all): python module deps
py_binary(
    name = "cyber_launch",
    srcs = ["cyber_launch.py"],
)

install_files(
    name = "install",
    dest = "bin",
    files = [
        ":cyber_launch.py",
    ],
    rename = {
        "bin/cyber_launch.py": "cyber_launch",
    },
)

