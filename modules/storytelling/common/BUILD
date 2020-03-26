load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "storytelling_gflags",
    srcs = ["storytelling_gflags.cc"],
    hdrs = ["storytelling_gflags.h"],
    copts = [
        "-DMODULE_NAME=\\\"storytelling\\\"",
    ],
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
