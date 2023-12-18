load("//tools/proto:proto.bzl", "apollo_py_binary")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_py_binary(
    name = "evaluate_compare",
    srcs = ["evaluate_compare.py"],
)

# install(
#     name = "install",
#     py_dest = "tools/localization",
#     targets = [
#         ":evaluate_compare",
#     ]

# )

apollo_package()