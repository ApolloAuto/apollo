load("//tools/install:install.bzl", "install", "install_files", "install_src_files")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install",
    data_dest = "common-msgs",
    data = [
        ":cyberfile.xml",
    ],
)

install_src_files(
    name = "install_src",
    src_dir = ["."],
    dest = "common-msgs/src",
    filter = "*",
)
