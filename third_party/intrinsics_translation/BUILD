load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools/install:install.bzl", "install", "install_files", "install_src_files")

package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "intrinsics_translation",
    hdrs = [
        "sse2neon/sse2neon.h"
    ],
    copts = select(
        {
            "@platforms//cpu:x86_64": [],
            "@platforms//cpu:aarch64": [
                "-march=armv8-a+fp+simd+crc",
            ],
	    },
        no_match_error = "Please Build with an ARM or Linux x86_64 platform",
    ),
)

install(
    name = "install",
    data_dest = "3rd-intrinsics-translation",
    data = [
        "cyberfile.xml",
    ],
)

install_src_files(
    name = "install_src",
    src_dir = ["."],
    dest = "3rd-intrinsics-translation/src",
    filter = "*",
)

