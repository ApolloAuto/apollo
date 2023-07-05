load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data_dest = "tools/map_datachecker",
    data = [":scripts"],
    visibility = ["//visibility:public"],
)

filegroup(
    name = "scripts",
    srcs = glob([
        "*.sh",
    ]),
)
