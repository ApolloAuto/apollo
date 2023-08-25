load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
# load("//tools:apollo_package.bzl", "apollo_package")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "hermes_can",
    srcs = select(
        {
            "@platforms//cpu:x86_64": glob([
                "lib_x86_64/libbcan.so",
            ]),
            "@platforms//cpu:aarch64": glob([
                "lib_aarch64/libbcan.so",
            ]),
        },
        no_match_error = "Please Build with a Linux aarch64 or x86_64 platform",
    ),
    hdrs = [
        "include/bcan.h",
    ],
    alwayslink = True,
    include_prefix = "hermes_can",
    tags = ["shared_library"],
)

cc_library(
    name = "adv_trigger",
    srcs = select(
        {
            "@platforms//cpu:x86_64": glob([
                "lib_x86_64/libadv*",
            ]),
            "@platforms//cpu:aarch64": glob([
                "lib_aarch64/libadv*",
            ]),
        },
        no_match_error = "Please Build with a Linux aarch64 or x86_64 platform",
    ),
    hdrs = [
        "include/bcan.h",
    ],
    alwayslink = True,
    include_prefix = "hermes_can",
    tags = ["shared_library"],
)

install_src_files(
    name = "lib",
    src_dir = select(
        {
            "@platforms//cpu:x86_64": [
                "lib_x86_64",
            ],
            "@platforms//cpu:aarch64": [
                "lib_aarch64",
            ],
        },
        no_match_error = "Please Build with a Linux aarch64 or x86_64 platform",
    ),
    dest = "3rd-can-card-library/lib",
    filter = "*.so*",
)

install_src_files(
    name = "headers",
    src_dir = ["include"],
    dest = "3rd-can-card-library/include",
    filter = "*",
)

# apollo_package()
