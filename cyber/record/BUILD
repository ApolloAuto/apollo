load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "record",
    deps = [
        "record_reader",
        "record_writer",
        "record_viewer",
    ],
)

cc_library(
    name = "record_file_base",
    hdrs = [ "file/record_file_base.h", ],
    deps = [
        "//cyber/proto:record_cc_proto",
    ],
)

cc_library(
    name = "record_file_reader",
    srcs = [ "file/record_file_reader.cc", ],
    hdrs = [ "file/record_file_reader.h", ],
    deps = [
        "record_file_base",
        "section",
        "//cyber/common:log",
        "//cyber/common:file",
    ],
)

cc_library(
    name = "record_file_writer",
    srcs = [ "file/record_file_writer.cc", ],
    hdrs = [ "file/record_file_writer.h", ],
    deps = [
        "record_file_base",
        "section",
        "//cyber/common:log",
        "//cyber/common:file",
    ],
)

cc_library(
    name = "section",
    hdrs = [ "file/section.h", ],
)

cc_test(
    name = "record_file_test",
    size = "small",
    srcs = [ "file/record_file_test.cc", ],
    deps = [
        "//cyber",
        "//cyber/proto:record_cc_proto",
        "@gtest//:main",
    ],
)

cc_library(
    name = "header_builder",
    srcs = [ "header_builder.cc", ],
    hdrs = [ "header_builder.h", ],
    deps = [
        "//cyber/proto:record_cc_proto",
    ],
)

cc_library(
    name = "record_base",
    hdrs = [ "record_base.h", ],
    deps = [
        "//cyber/proto:record_cc_proto",
    ],
)

cc_library(
    name = "record_message",
    hdrs = [ "record_message.h", ],
)

cc_library(
    name = "record_reader",
    srcs = [ "record_reader.cc", ],
    hdrs = [ "record_reader.h", ],
    deps = [
        "record_base",
        "record_file_reader",
        "record_message",
    ],
)

cc_library(
    name = "record_viewer",
    srcs = [ "record_viewer.cc", ],
    hdrs = [ "record_viewer.h", ],
    deps = [
        "record_message",
        "record_reader",
    ],
)

cc_library(
    name = "record_writer",
    srcs = [ "record_writer.cc", ],
    hdrs = [ "record_writer.h", ],
    deps = [
        "record_base",
        "header_builder",
        "record_file_writer",
        "//cyber/message:raw_message",
        "//cyber/proto:record_cc_proto",
    ],
)

cpplint()
