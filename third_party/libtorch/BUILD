load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("@rules_cc//cc:defs.bzl", "cc_library")

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
    deps = [
        ":install_cpu",
        ":install_gpu"
    ]
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_cpu_src",
        ":install_gpu_src"
    ]
)

install(
    name = "install_cpu",
    data_dest = "3rd-libtorch-cpu",
    data = [
        ":cyberfile-cpu.xml",
        ":3rd-libtorch-cpu.BUILD",
        ":3rd-libtorch.BUILD",
    ],
    rename = {
        "3rd-libtorch-cpu/cyberfile-cpu.xml" : "cyberfile.xml",
        "3rd-libtorch-cpu/3rd-libtorch.BUILD" : "BUILD",
    },
)

install_src_files(
    name = "install_cpu_src",
    src_dir = ["."],
    dest = "3rd-libtorch-cpu/src",
    filter = "*",
)

install(
    name = "install_gpu",
    data_dest = "3rd-libtorch-gpu",
    data = [
        ":cyberfile-gpu.xml",
        ":3rd-libtorch-gpu.BUILD",
        ":3rd-libtorch.BUILD",
    ],
    rename = {
        "3rd-libtorch-gpu/cyberfile-gpu.xml" : "cyberfile.xml",
        "3rd-libtorch-gpu/3rd-libtorch.BUILD" : "BUILD",
    },
)

install_src_files(
    name = "install_gpu_src",
    src_dir = ["."],
    dest = "3rd-libtorch-gpu/src",
    filter = "*",
)