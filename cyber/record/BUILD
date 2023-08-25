load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_cc_test")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "cyber_record",
    srcs = [
        "header_builder.cc",
        "record_reader.cc",
        "record_viewer.cc",
        "record_writer.cc",
        "file/record_file_base.cc",
        "file/record_file_reader.cc",
        "file/record_file_writer.cc",
    ],
    hdrs = [
        "header_builder.h",
        "record_base.h",
        "record_message.h",
        "record_reader.h",
        "record_viewer.h",
        "record_writer.h",
        "file/record_file_base.h",
        "file/record_file_reader.h",
        "file/record_file_writer.h",
        "file/section.h",
    ],
    deps = [
        "//cyber/common:cyber_common",
        "//cyber/proto:record_cc_proto",
        "//cyber/time:cyber_time",
        "@com_google_protobuf//:protobuf",
        "//cyber/message:cyber_message",
    ],
)


apollo_cc_test(
    name = "record_file_test",
    size = "small",
    srcs = ["file/record_file_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:record_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
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

apollo_cc_test(
    name = "record_reader_test",
    size = "small",
    srcs = ["record_reader_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:record_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "record_viewer_test",
    size = "small",
    srcs = ["record_viewer_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:record_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()
cpplint()
