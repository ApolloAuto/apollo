load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")

package(
    default_visibility = ["//visibility:public"],
)

cc_library(
    name = "tf2",
    srcs = [
        "src/buffer_core.cpp",
        "src/cache.cpp",
        "src/static_cache.cpp",
        "src/time.cpp",
    ],
    hdrs = glob([
        "include/geometry_msgs/**",
        "include/tf2_msgs/**",
        "include/tf2/**",
    ]),
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
    deps = [
        "@boost",
    ],
)

install(
    name = "install",
    data_dest = "3rd-tf2",
    library_dest = "3rd-tf2/lib",
    targets = ["tf2"],
    data = [
        ":cyberfile.xml",
        ":3rd-tf2.BUILD",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_tf2_src",
        ":install_tf2_hdrs"
    ],
)

install_src_files(
    name = "install_tf2_src",
    src_dir = ["."],
    dest = "3rd-tf2/src",
    filter = "*",
)

install_src_files(
    name = "install_tf2_hdrs",
    src_dir = ["."],
    dest = "3rd-tf2",
    filter = "*.h",
)