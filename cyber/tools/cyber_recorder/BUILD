load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "cyber_recorder",
    srcs = ["main.cc"],
    linkopts = ["-pthread"],
    deps = [
        ":info",
        ":recorder",
        ":recoverer",
        ":spliter",
        "//cyber:init",
        "//cyber/common:file",
        "//cyber/common:time_conversion",
        "//cyber/tools/cyber_recorder/player",
    ],
)

cc_library(
    name = "info",
    srcs = ["info.cc"],
    hdrs = ["info.h"],
    deps = [
        "//cyber/common:time_conversion",
        "//cyber/proto:record_cc_proto",
        "//cyber/record:record_file_reader",
        "//cyber/record:record_message",
    ],
)

cc_library(
    name = "recorder",
    srcs = ["recorder.cc"],
    hdrs = ["recorder.h"],
    deps = [
        "//cyber",
        "//cyber/common:log",
        "//cyber/proto:record_cc_proto",
        "@fastrtps",
    ],
)

cc_library(
    name = "recoverer",
    srcs = ["recoverer.cc"],
    hdrs = ["recoverer.h"],
    deps = [
        "//cyber/base:for_each",
        "//cyber/common:log",
        "//cyber/proto:record_cc_proto",
        "//cyber/record:header_builder",
        "//cyber/record:record_file_reader",
        "//cyber/record:record_writer",
    ],
)

cc_library(
    name = "spliter",
    srcs = ["spliter.cc"],
    hdrs = ["spliter.h"],
    deps = [
        "//cyber/common:log",
        "//cyber/proto:record_cc_proto",
        "//cyber/record:header_builder",
        "//cyber/record:record_file_reader",
        "//cyber/record:record_writer",
    ],
)

cpplint()
