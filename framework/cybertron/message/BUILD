load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "message_lib",
    srcs = glob([
        "protobuf_factory.cpp",
        "protobuf_factory.cpp",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//cybertron/common:common_lib",
        "//cybertron/proto:proto_desc_cc_proto",
    ],
)

cc_test(
    name = "raw_message_test",
    size = "small",
    srcs = [
        "raw_message_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_test(
    name = "message_traits_test",
    size = "small",
    srcs = [
        "message_traits_test.cpp",
    ],
    deps = [
        "//cybertron",
        "//cybertron/proto:unit_test_cc_proto",
        "@gtest//:main",
    ],
)

cpplint()
