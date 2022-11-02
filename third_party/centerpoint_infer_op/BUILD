load("//tools/install:install.bzl", "install", "install_files", "install_src_files")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install_lib",
    data_dest = "3rd-centerpoint-infer-op/lib",
    data = [
        ":libpd_infer_custom_op.so"
    ]
)

install(
    name = "install",
    data_dest = "3rd-centerpoint-infer-op",
    data = [
        ":cyberfile.xml",
    ],
    deps = ["install_lib"]
)

install_src_files(
    name = "install_src",
    src_dir = ["."],
    dest = "3rd-centerpoint-infer-op/src",
    filter = "*",
)
