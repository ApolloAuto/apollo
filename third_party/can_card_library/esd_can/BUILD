load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools/platform:build_defs.bzl", "if_esd_can")
package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "esd_can",
    srcs = if_esd_can(["lib/libntcan.so.4"]),
    hdrs = if_esd_can(["include/ntcan.h"]),
    include_prefix = "esd_can",
)
