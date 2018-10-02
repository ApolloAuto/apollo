#load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "proto_lib",
    srcs = glob([
        "*.pb.cc",
    ]),
    hdrs = glob([
        "*.h",
        "*.pb.h",
    ]),
    copts = ["-Iframework"],
)

#cpplint()
