load("//tools/install:install.bzl", "install", "install_files", "install_src_files")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install_lib",
    data_dest = "3rd-caddn-infer-op/lib",
    data = glob([
        "lib*.so"
    ])
)

install(
    name = "install",
    data_dest = "3rd-caddn-infer-op",
    data = [
        ":cyberfile.xml",
    ],
    deps = ["install_lib"]
)

install_src_files(
    name = "install_src",
    src_dir = ["."],
    dest = "3rd-caddn-infer-op/src",
    filter = "*",
)
