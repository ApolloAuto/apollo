load("@rules_python//python:defs.bzl", "py_binary")
# load("//tools/install:install.bzl", "install_files")
load("//tools:apollo_package.bzl", "apollo_package")

package(
    default_visibility = ["//visibility:public"],
)

# FIXME(all): python module deps
py_binary(
    name = "cyber_launch",
    srcs = ["cyber_launch.py"],
)

# Note(storypku):
# Workaround for install support of Python not ready.
# install_files(
#     name = "install",
#     dest = "cyber/bin",
#     files = [
#         ":cyber_launch.py",
#     ],
#     rename = {
#         "cyber/bin/cyber_launch.py": "cyber_launch",
#     },
# )

apollo_package()