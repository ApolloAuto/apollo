load("//tools/install:install.bzl", "install")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install",
    data = [
        ":base_files",
        ":docker_files",
    ],
)

filegroup(
    name = "docker_files",
    srcs = [
        ":docker_start_user.sh",
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
