load("//tools/install:install.bzl", "install")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install",
    data = [
        ":base_files"
    ],
)

filegroup(
    name = "base_files",
    srcs = [
        ":apollo.bashrc",
        ":common.bashrc",
        ":apollo_base.sh",
    ],
)
