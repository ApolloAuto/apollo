load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "history_attributes",
    hdrs = ["history_attributes.h"],
    deps = [
        "//cyber/proto:qos_profile_cc_proto",
    ],
)

cc_library(
    name = "history",
    hdrs = ["history.h"],
    deps = [
        ":history_attributes",
    ],
)

cc_library(
    name = "listener_handler",
    hdrs = ["listener_handler.h"],
    deps = [
        ":message_info",
        "//cyber/base:signal",
        "//cyber/message:message_traits",
        "//cyber/message:raw_message",
    ],
)

cc_library(
    name = "message_info",
    srcs = ["message_info.cc"],
    hdrs = ["message_info.h"],
    deps = [
        "//cyber/common:log",
        "//cyber/transport/common:identity",
    ],
)

cc_test(
    name = "message_info_test",
    size = "small",
    srcs = ["message_info_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "message_test",
    size = "small",
    srcs = ["message_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
