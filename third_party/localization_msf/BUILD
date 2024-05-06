load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

install(
    name = "install_lib",
    data = glob([
        "lib*",
    ]),
    data_dest = "3rd-localization-msf/lib",
)

install(
    name = "install",
    data_dest = "3rd-localization-msf",
    data = [
        "cyberfile.xml",
    ],
)

install_src_files(
    name = "install_src",
    src_dir = ["."],
    dest = "3rd-localization-msf/src",
    filter = "*",
)
