load("@rules_python//python:defs.bzl", "py_binary")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(
    default_visibility = ["//visibility:public"],
)

py_binary(
    name = "cyber_node",
    srcs = ["cyber_node.py"],
    deps = [
        "//cyber/python/cyber_py3:cyber",
        "//cyber/proto:role_attributes_py_pb2"
    ],
)

# install(
#     name = "install",
#     py_dest = "cyber/bin",
#     targets = [":cyber_node"]
# )

apollo_package()