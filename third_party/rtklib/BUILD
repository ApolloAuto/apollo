load("//tools/install:install.bzl", "install", "install_files", "install_src_files")

load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rtklib",
    srcs = [
        "novatel.c",
        "rcvraw.c",
        "rtcm.c",
        "rtcm3.c",
        "rtkcmn.c",
    ],
    linkstatic = True,
    alwayslink = True,
    hdrs = ["rtklib.h"],
)

cc_binary(
    name = "librtklib.so",
    linkshared = True,
    linkstatic = True,
    deps = [":rtklib"]
)

install(
    name = "install",
    data_dest = "3rd-rtklib",
    library_dest = "3rd-rtklib/lib",
    data = [
        ":cyberfile.xml",
        ":3rd-rtklib.BUILD"
    ],
    targets = ["librtklib.so"]
)

install_src_files(
    name = "headers",
    src_dir = ["."],
    dest = "3rd-rtklib/include",
    filter = "*.h",
)


install_src_files(
    name = "install_src",
    src_dir = ["."],
    dest = "3rd-rtklib/src",
    filter = "*",
    deps = [
        ":headers",
    ]
)