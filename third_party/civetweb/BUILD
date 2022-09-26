load("//tools/install:install.bzl", "install", "install_files", "install_src_files")

package(
    default_visibility = ["//visibility:public"],
)

install(
    name = "install",
    data_dest = "3rd-civetweb",
    library_dest = "3rd-civetweb/lib",
    data = [
        ":cyberfile.xml",
    ],
    deps = [
        ":install_libs",
        ":install_hdrs",
    ],
)

install(
    name = "install_libs",
    data_dest = "3rd-civetweb/lib",
    data = [
        "@civetweb//:civetweb_libs",
    ],
    data_strip_prefix = [
        "third_party/civetweb",
    ],

)

install(
    name = "install_hdrs",
    data_dest = "3rd-civetweb/include",
    data = [
        "@civetweb//:civetweb_hdrs",
    ],
    data_strip_prefix = [
        "third_party/civetweb",
    ],

)

install_src_files(
    name = "install_src",
    src_dir = ["."],
    dest = "3rd-civetweb/src",
    filter = "*",
)