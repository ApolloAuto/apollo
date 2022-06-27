load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dreamview_gflags",
    srcs = ["dreamview_gflags.cc"],
    hdrs = ["dreamview_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "curl_request",
    srcs = ["curl_request.cc"],
    hdrs = ["curl_request.h"],
    deps = [
        "//cyber",
        "@boost",
    ],
)

cpplint()
