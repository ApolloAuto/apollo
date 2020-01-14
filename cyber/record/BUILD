load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "record",
    deps = [
        ":record_reader",
        ":record_viewer",
        ":record_writer",
    ],
)

cc_library(
    name = "record_file_base",
    srcs = ["file/record_file_base.cc"],
    hdrs = ["file/record_file_base.h"],
    deps = [
        "//cyber/common:log",
        "//cyber/proto:record_cc_proto",
    ],
)

cc_library(
    name = "record_file_reader",
    srcs = ["file/record_file_reader.cc"],
    hdrs = ["file/record_file_reader.h"],
    deps = [
        ":record_file_base",
        ":section",
        "//cyber/common:file",
        "//cyber/time",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "record_file_writer",
    srcs = ["file/record_file_writer.cc"],
    hdrs = ["file/record_file_writer.h"],
    deps = [
        ":record_file_base",
        ":section",
        "//cyber/common:file",
        "//cyber/time",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_library(
    name = "section",
    hdrs = ["file/section.h"],
)

cc_test(
    name = "record_file_test",
    size = "small",
    srcs = ["file/record_file_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:record_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "record_file_integration_test",
    size = "small",
    srcs = ["file/record_file_integration_test.cc"],
    tags = [
        "cpu:3",
        "exclusive",
        "manual",
    ],
    deps = [
        "//cyber",
        "//cyber/proto:record_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "header_builder",
    srcs = ["header_builder.cc"],
    hdrs = ["header_builder.h"],
    deps = [
        "//cyber/proto:record_cc_proto",
    ],
)

cc_library(
    name = "record_base",
    hdrs = ["record_base.h"],
    deps = [
        "//cyber/proto:record_cc_proto",
    ],
)

cc_library(
    name = "record_message",
    hdrs = ["record_message.h"],
)

cc_library(
    name = "record_reader",
    srcs = ["record_reader.cc"],
    hdrs = ["record_reader.h"],
    deps = [
        ":record_base",
        ":record_file_reader",
        ":record_message",
    ],
)

cc_test(
    name = "record_reader_test",
    size = "small",
    srcs = ["record_reader_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:record_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "record_viewer",
    srcs = ["record_viewer.cc"],
    hdrs = ["record_viewer.h"],
    deps = [
        ":record_message",
        ":record_reader",
    ],
)

cc_test(
    name = "record_viewer_test",
    size = "small",
    srcs = ["record_viewer_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:record_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "record_writer",
    srcs = ["record_writer.cc"],
    hdrs = ["record_writer.h"],
    deps = [
        ":header_builder",
        ":record_base",
        ":record_file_writer",
        "//cyber/message:message_traits",
        "//cyber/message:raw_message",
        "//cyber/proto:record_cc_proto",
    ],
)

cpplint()
