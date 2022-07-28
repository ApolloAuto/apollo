load("//tools/install:install.bzl", "install", "install_files", "install_src_files")

package(
    default_visibility = ["//visibility:public"],
)
cc_library(
    name = "libtorch",
    deps = select({
        "//tools/platform:use_gpu": [
            "@libtorch_gpu",
        ],
        "//conditions:default": [
            "@libtorch_cpu",
        ],
    }),
)

install(
    name = "install",
    data_dest = "3rd-libtorch-cpu",
    data = [
        ":cyberfile-cpu.xml",
        ":3rd-libtorch-cpu.BUILD",
    ],
    rename = {
        "3rd-libtorch-cpu/cyberfile-cpu.xml" : "cyberfile.xml",
    },
)

install_src_files(
    name = "install_src",
    src_dir = ["."],
    dest = "3rd-libtorch-cpu/src",
    filter = "*",
)