load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "perception",
    data = [
        ":runtime_data",
    ],
)

filegroup(
    name = "runtime_data",
    srcs = glob([
        "params/*.yaml",
    ]),
)
