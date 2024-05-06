load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
# load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "smartereye",
    srcs = glob(["lib/*.so*"]),
    hdrs = glob(["include/*.h"]),
    copts = [
        "-Iinclude",
    ],
    includes = [
        "include",
    ],
    linkopts = [
        "-Llib",
    ],
    visibility = ["//visibility:public"],
)

install_src_files(
    name = "headers",
    src_dir = ["include"],
    dest = "3rd-camera-library/include",
    filter = "*",
)
# apollo_package()