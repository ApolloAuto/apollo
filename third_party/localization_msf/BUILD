load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools/platform:build_defs.bzl", "if_aarch64", "if_x86_64")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])



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
    deps = [":install_src_lib"]
)

install_src_files(
    name = "install_src_lib",
    src_dir = if_x86_64([
        "x86_64/lib",
    ]) + if_aarch64([
        "aarch64/lib",
    ]),
    dest = "3rd-localization-msf/lib",
    filter = "*",
)
