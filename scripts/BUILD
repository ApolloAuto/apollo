load("//tools/install:install.bzl", "install", "install_files")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install",
    deps = [
        ":install_bashrc",
    ],
)

install_files(
    name = "install_bashrc",
    dest = "scripts",
    files = [
        ":apollo.bashrc",
        ":common.bashrc",
    ],
)
