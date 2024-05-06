load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    data = [
        ":runtime",
    ],
)

filegroup(
    name = "runtime",
    srcs = glob([
        "*.sh",
    ]),
)
