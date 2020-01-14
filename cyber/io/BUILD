load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "io",
    deps = [
        ":poll_data",
        ":poll_handler",
        ":poller",
        ":session",
    ],
)

cc_library(
    name = "poll_data",
    hdrs = ["poll_data.h"],
)

cc_library(
    name = "poll_handler",
    srcs = ["poll_handler.cc"],
    hdrs = ["poll_handler.h"],
    deps = [
        ":poll_data",
        ":poller",
        "//cyber/common:log",
        "//cyber/croutine",
    ],
)

cc_library(
    name = "poller",
    srcs = ["poller.cc"],
    hdrs = ["poller.h"],
    deps = [
        ":poll_data",
        "//cyber/base:atomic_rw_lock",
        "//cyber/common:log",
        "//cyber/common:macros",
        "//cyber/scheduler:scheduler_factory",
        "//cyber/time",
    ],
)

cc_test(
    name = "poller_test",
    size = "small",
    srcs = ["poller_test.cc"],
    deps = [
        ":poller",
        "//cyber:cyber_core",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "session",
    srcs = ["session.cc"],
    hdrs = ["session.h"],
    deps = [
        ":poll_handler",
        "//cyber/common:log",
    ],
)

cc_binary(
    name = "tcp_echo_client",
    srcs = ["example/tcp_echo_client.cc"],
    deps = [
        "//cyber:cyber_core",
    ],
)

cc_binary(
    name = "tcp_echo_server",
    srcs = ["example/tcp_echo_server.cc"],
    deps = [
        "//cyber:cyber_core",
    ],
)

cc_binary(
    name = "udp_echo_client",
    srcs = ["example/udp_echo_client.cc"],
    deps = [
        "//cyber:cyber_core",
    ],
)

cc_binary(
    name = "udp_echo_server",
    srcs = ["example/udp_echo_server.cc"],
    deps = [
        "//cyber:cyber_core",
    ],
)

cpplint()
