load("//tools/install:install.bzl", "install", "install_files", "install_src_files")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install_lib",
    data = glob([
        "lib*.so",
    ]),
    data_dest = "3rd-centerpoint-infer-op/lib",
)

install(
    name = "install",
    data = [
        ":cyberfile.xml",
    ],
    data_dest = "3rd-centerpoint-infer-op",
    deps = ["install_lib"],
)

install_src_files(
    name = "install_src",
    dest = "3rd-centerpoint-infer-op/src",
    filter = "*",
    src_dir = ["."],
)
