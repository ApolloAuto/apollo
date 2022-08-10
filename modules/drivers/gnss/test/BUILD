load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    runtime_dest = "drivers/bin",
    targets = [
      ":parser_cli",
    ],
)

cc_binary(
    name = "parser_cli",
    srcs = ["parser_cli.cc"],
    deps = [
        "//cyber",
        "//modules/drivers/gnss/proto:gnss_status_cc_proto",
        "//modules/drivers/gnss/stream:gnss_stream",
        "@com_github_gflags_gflags//:gflags",
    ],
)

cpplint()
