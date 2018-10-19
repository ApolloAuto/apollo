load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "record",
    deps = [
        "header_builder",
        "record_file",
        "record_reader",
        "record_writer",
        "record_base",
        "record_message",
        "record_viewer",
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
    name = "record_file",
    hdrs = [ "record_file.h", ],
    srcs = [ "record_file.cc", ],
    deps = [
        "//cyber/common:file",
        "//cyber/common:log",
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
        "record_file",
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
        "header_builder",
        "record_base",
        "record_file",
        "//cyber/message:raw_message",
        "//cyber/proto:record_cc_proto",
    ],
)

cpplint()
