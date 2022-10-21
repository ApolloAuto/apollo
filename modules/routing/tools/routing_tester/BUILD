load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

package(default_visibility = ["//visibility:public"])

install(
    name = "install",
    runtime_dest = "routing/bin",
    targets = [
        ":routing_tester",
    ],
)
cc_binary(
    name = "routing_tester",
    srcs = ["routing_tester.cc"],
    data = [
        ":test_data",
    ],
    deps = [
        "//modules/routing",
    ],
)

filegroup(
    name = "test_data",
    srcs = glob([
        "*.pb.txt",
    ]),
)

cpplint()
