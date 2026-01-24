load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
# load("//tools:apollo_package.bzl", "apollo_package")

package(
    default_visibility = ["//visibility:public"],
)

cc_binary(
    name = "libtf2.so",
    srcs = [
        "src/buffer_core.cpp",
        "src/cache.cpp",
        "src/static_cache.cpp",
        "src/time.cpp",
    ],
    deps = [
        "@boost",
        ":tf2_headers",
    ],
    linkshared = True,
)

cc_library(
    name = "tf2_headers",
    hdrs = glob([
        "include/geometry_msgs/**",
        "include/tf2_msgs/**",
        "include/tf2/**",
    ]),
    strip_include_prefix = "include",
)

cc_library(
    name = "tf2",
    srcs = [
        ":libtf2.so",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@boost",
        ":tf2_headers",
    ],
)

install(
    name = "install",
    data_dest = "3rd-tf2",
    library_dest = "3rd-tf2/lib",
    targets = [
        ":libtf2.so"
    ],
    data = [
        ":cyberfile.xml",
        ":3rd-tf2.BUILD",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_tf2_src",
        ":install_tf2_hdrs",
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
    src_dir = ["include"],
    dest = "3rd-tf2/include",
    filter = "*",
)
# apollo_package()
