load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "transport",
    srcs = ["transport.cc"],
    hdrs = ["transport.h"],
    deps = [
        "//cyber/service_discovery:role",
        "//cyber/task",
        "//cyber/transport/dispatcher:intra_dispatcher",
        "//cyber/transport/dispatcher:rtps_dispatcher",
        "//cyber/transport/dispatcher:shm_dispatcher",
        "//cyber/transport/message:history",
        "//cyber/transport/qos",
        "//cyber/transport/receiver",
        "//cyber/transport/rtps:attributes_filler",
        "//cyber/transport/rtps:participant",
        "//cyber/transport/rtps:sub_listener",
        "//cyber/transport/rtps:underlay_message",
        "//cyber/transport/rtps:underlay_message_type",
        "//cyber/transport/transmitter",
        "@fastrtps",
    ],
)

cc_test(
    name = "transport_test",
    size = "small",
    srcs = ["transport_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
