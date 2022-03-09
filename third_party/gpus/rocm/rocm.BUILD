load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "rocm_headers",
    hdrs = glob([
        "include/*.h",
    ]),
    includes = [
        "include",
    ],
)

cc_library(
    name = "hip",
    srcs = ["lib/libamdhip64.so"],
    data = ["lib/libamdhip64.so"],
    includes = [
        "include/",
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "miopen",
    srcs = ["lib/libMIOpen.so"],
    data = ["lib/libMIOpen.so"],
    includes = [
        "include/",
    ],
    linkstatic = 1,
    visibility = ["//visibility:public"],
)

cc_library(
    name = "hipblas",
    srcs = ["lib/libhipblas.so"],
    data = ["lib/libhipblas.so"],
)