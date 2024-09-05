load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_binary", "apollo_cc_library", "apollo_cc_test", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_cc_test(
    name = "message_header_test",
    size = "small",
    srcs = ["message_header_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_library(
    name = "cyber_message",
    srcs = [
        "protobuf_factory.cc",
    ],
    hdrs = [
        "arena_manager_base.h",
        "arena_message_wrapper.h",
        "message_header.h",
        "message_traits.h",
        "protobuf_factory.h",
        "protobuf_traits.h",
        "py_message.h",
        "py_message_traits.h",
        "raw_message.h",
        "raw_message_traits.h",
    ],
    deps = [
        "//cyber/common:cyber_common",
        "//cyber/proto:proto_desc_cc_proto",
    ],
)

apollo_cc_test(
    name = "message_traits_test",
    size = "small",
    srcs = ["message_traits_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "raw_message_test",
    size = "small",
    srcs = ["raw_message_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "protobuf_factory_test",
    size = "small",
    srcs = ["protobuf_factory_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
