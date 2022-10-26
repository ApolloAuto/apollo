load("//tools/install:install.bzl", "install", "install_files", "install_src_files")

package(
    default_visibility = ["//visibility:public"],
)

exports_files(["CPPLINT.cfg"])

install(
    name = "install",
    data_dest = "bazel-extend-tools",
    data = [
        ":cyberfile.xml",
    ],
)

install_src_files(
    name = "install_src",
    src_dir = ["."],
    dest = "bazel-extend-tools/src",
    filter = "*",
)
