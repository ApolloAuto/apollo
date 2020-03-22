load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "message_header",
    hdrs = ["message_header.h"],
)

cc_test(
    name = "message_header_test",
    size = "small",
    srcs = ["message_header_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "message_traits",
    hdrs = ["message_traits.h"],
    deps = [
        ":message_header",
        ":protobuf_traits",
        ":py_message_traits",
        ":raw_message_traits",
        "//cyber/base:macros",
    ],
)

cc_test(
    name = "message_traits_test",
    size = "small",
    srcs = ["message_traits_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "protobuf_factory",
    srcs = ["protobuf_factory.cc"],
    hdrs = ["protobuf_factory.h"],
    deps = [
        "//cyber/common:log",
        "//cyber/common:macros",
        "//cyber/proto:proto_desc_cc_proto",
    ],
)

cc_library(
    name = "protobuf_traits",
    hdrs = ["protobuf_traits.h"],
    deps = [
        ":protobuf_factory",
    ],
)

cc_library(
    name = "py_message",
    hdrs = ["py_message.h"],
    deps = [
        ":protobuf_factory",
    ],
)

cc_library(
    name = "py_message_traits",
    hdrs = ["py_message_traits.h"],
    deps = [
        ":protobuf_factory",
        ":py_message",
    ],
)

cc_library(
    name = "raw_message",
    hdrs = ["raw_message.h"],
    deps = [
        ":protobuf_factory",
    ],
)

cc_test(
    name = "raw_message_test",
    size = "small",
    srcs = ["raw_message_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "protobuf_factory_test",
    size = "small",
    srcs = ["protobuf_factory_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "raw_message_traits",
    hdrs = ["raw_message_traits.h"],
    deps = [
        ":protobuf_factory",
        ":raw_message",
    ],
)

cpplint()
