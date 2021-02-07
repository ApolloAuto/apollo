load("@rules_cc//cc:defs.bzl", "cc_binary")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

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
