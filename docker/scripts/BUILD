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
    srcs = [
        "docker_base.sh",
        "runtime_start.sh",
        "runtime_into.sh",
        "runtime_into_standalone.sh",
    ],
)
