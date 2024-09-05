load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
# load("//tools:apollo_package.bzl", "apollo_package")

package(
    default_visibility = ["//visibility:public"],
)

cc_binary(
    name = "libapollo_tf2.so",
    srcs = [
        "src/buffer_core.cpp",
        "src/cache.cpp",
        "src/static_cache.cpp",
        "src/time.cpp",
    ],
    linkshared = True,
    deps = [
        ":tf2_headers",
        "@boost",
    ],
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
        ":libapollo_tf2.so",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":tf2_headers",
        "@boost",
    ],
)

install(
    name = "install",
    data = [
        ":3rd-tf2.BUILD",
        ":cyberfile.xml",
    ],
    data_dest = "3rd-tf2",
    library_dest = "3rd-tf2/lib",
    targets = [
        ":libapollo_tf2.so",
    ],
)

install_src_files(
    name = "install_src",
    deps = [
        ":install_tf2_hdrs",
        ":install_tf2_src",
    ],
)

install_src_files(
    name = "install_tf2_src",
    dest = "3rd-tf2/src",
    filter = "*",
    src_dir = ["."],
)

install_src_files(
    name = "install_tf2_hdrs",
    dest = "3rd-tf2/include",
    filter = "*",
    src_dir = ["include"],
)
# apollo_package()
