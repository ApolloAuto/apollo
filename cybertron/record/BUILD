load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "record_lib",
    srcs = glob([
        "header_builder.cc",
        "record_file.cc",
        "record_reader.cc",
        "record_viewer.cc",
        "record_writer.cc",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//cybertron:init",
        "//cybertron/common:common",
        "//cybertron/message:raw_message",
        "//cybertron/proto:record_cc_proto",
    ],
)

cpplint()
