load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "v2x_proxy_gflags",
    srcs = ["v2x_proxy_gflags.cc"],
    hdrs = ["v2x_proxy_gflags.h"],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
