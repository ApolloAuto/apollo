load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "parser_cli",
    srcs = ["parser_cli.cc"],
    deps = [
        "//cyber",
        "//modules/drivers/gnss/stream:gnss_stream",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
