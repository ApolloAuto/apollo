load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "record_lib",
    srcs = glob([
        "header_builder.cpp",
        "record_file.cpp",
        "record_reader.cpp",
        "record_viewer.cpp",
        "record_writer.cpp",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//cybertron:init_lib",
        "//cybertron/common:common_lib",
        "//cybertron/message:message_lib",
        "//cybertron/proto:record_cc_proto",
    ],
)

cpplint()
