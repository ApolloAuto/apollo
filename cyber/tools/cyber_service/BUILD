load("@rules_python//python:defs.bzl", "py_binary")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(
    default_visibility = ["//visibility:public"],
)

py_binary(
    name = "cyber_service",
    srcs = ["cyber_service.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
    ],
)

# install(
#     name = "install",
#     py_dest = "cyber/bin",
#     targets = [":cyber_service"]
# )
apollo_package()