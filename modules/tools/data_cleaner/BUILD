load("//tools/proto:proto.bzl", "apollo_py_binary")
# load("//tools/install:install.bzl", "install")
load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "config",
    srcs = glob([
        "data_cleaner.yaml"
    ]),
)

apollo_py_binary(
    name = "data_cleaner",
    srcs = ["data_cleaner.py"],
    deps = [
        "//modules/tools/common:logger",
    ],
)

# install(
#     name = "install",
#     data = [":config"],
#     data_dest = "tools/data_cleaner",
#     py_dest = "tools/data_cleaner",
#     targets = [
#         ":data_cleaner"
#     ],
# )

apollo_package()