load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "hermes_can",
    hdrs = [
        "include/bcan.h",
    ],
    srcs = glob(["lib/libbcan.so"]),
    strip_include_prefix = "include",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "third_party_Scan_card_library_Shermes_can_Chermes_can",
    srcs = glob(["lib/libbcan.so"]),
    hdrs = [
        "include/bcan.h",
    ],
    visibility = ["//visibility:public"],
    alwayslink = True,
)

cc_library(
    name = "third_party_Scan_card_library_Sesd_can_Cesd_can",
    srcs = glob(["lib/libntcan.so.4"]),
    hdrs = glob(["include/ntcan.h"]),
    include_prefix = "esd_can",
    visibility = ["//visibility:public"],
    alwayslink = True,
)
