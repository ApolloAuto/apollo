load("@rules_cc//cc:defs.bzl", "cc_library")

licenses(["notice"])

package(default_visibility = ["//visibility:public"])

# TODO(all): May use rules_boost.
cc_library(
    name = "boost",
    includes = ["."],
    hdrs = glob(["**/*"]),
    linkopts = [
        "-L/opt/apollo/sysroot/lib",
        "-lboost_filesystem",
        "-lboost_program_options",
        "-lboost_regex",
        "-lboost_system",
        "-lboost_thread",
    ],
)
