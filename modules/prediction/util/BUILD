load("//tools:cpplint.bzl", "cpplint")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "data_extraction",
    srcs = ["data_extraction.cc"],
    hdrs = [
        "data_extraction.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"prediction\\\"",
    ],
    linkopts = [
        "-lboost_filesystem",
    ],
    deps = [
        "//cyber",
    ],
)

cpplint()
