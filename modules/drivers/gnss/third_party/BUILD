load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rtcm",
    srcs = [
        "novatel.c",
        "rcvraw.c",
        "rtcm.c",
        "rtcm3.c",
        "rtkcmn.c",
    ],
    hdrs = ["rtklib.h"],
)
