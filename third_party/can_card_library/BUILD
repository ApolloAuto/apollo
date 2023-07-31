load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:apollo_package.bzl", "apollo_package")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install",
    data_dest = "3rd-can-card-library",
    data = [
        ":cyberfile.xml",
        ":3rd-can-card-library.BUILD"
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        "//third_party/can_card_library/hermes_can:lib",
        "//third_party/can_card_library/hermes_can:headers",
        ":src",
    ]
)

install_src_files(
    name = "src",
    src_dir = ["."],
    dest = "3rd-can-card-library/src",
    filter = "*",
)

# apollo_package()