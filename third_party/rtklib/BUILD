# load("//tools/install:install.bzl", "install", "install_files", "install_src_files")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library")

load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "rtklib",
    srcs = [
        "novatel.c",
        "rcvraw.c",
        "rtcm.c",
        "rtcm3.c",
        "rtkcmn.c",
    ],
    hdrs = ["rtklib.h"],
)

apollo_package()
