load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "bridge_buffer",
    srcs = [
        "bridge_buffer.cc",
    ],
    hdrs = [
        "bridge_buffer.h",
    ],
)

cc_library(
    name = "bridge_gflags",
    srcs = [
        "bridge_gflags.cc",
    ],
    hdrs = [
        "bridge_gflags.h",
    ],
)

cc_library(
    name = "util",
    srcs = [
        "util.cc",
    ],
    hdrs = [
        "util.h",
    ],
    deps = [
        ":bridge_buffer",
        ":macro",
    ],
)

cc_library(
    name = "macro",
    hdrs = [
        "macro.h",
    ],
)

cc_library(
    name = "bridge_proto_serialized_buf",
    hdrs = [
        "bridge_proto_serialized_buf.h",
    ],
)

cc_library(
    name = "bridge_proto_diserialized_buf",
    hdrs = [
        "bridge_proto_diserialized_buf.h",
    ],
)

cc_library(
    name = "udp_listener",
    hdrs = [
        "udp_listener.h",
    ],
)

cc_library(
    name = "bridge_header",
    srcs = [
        "bridge_header.cc",
    ],
    hdrs = [
        "bridge_header.h",
        "bridge_header_item.h",
    ],
)

cc_test(
    name = "bridge_buffer_test",
    size = "small",
    srcs = [
        "bridge_buffer_test.cc",
    ],
    deps = [
        ":bridge_buffer",
        "@gtest//:main",
    ],
)

cc_test(
    name = "bridge_proto_buf_test",
    size = "small",
    srcs = [
        "bridge_proto_buf_test.cc",
    ],
    deps = [
        "//modules/bridge:udp_bridge",
        "@gtest//:main",
    ],
)

cpplint()
