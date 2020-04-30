licenses(["notice"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "boost",
    copts = ["-fPIC"],
    includes = ["."],
    linkopts = [
        "-lboost_system",
        "-lboost_filesystem",
        "-lboost_program_options",
        "-lboost_thread",
        "-lboost_signals",
    ],
)
