load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_cc_test", "apollo_cc_binary")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "cyber_io",
    hdrs = [
        "poll_data.h",
        "poll_handler.h",
        "poller.h",
        "session.h",
    ],
    srcs = ["poll_handler.cc", "poller.cc", "session.cc"],
    deps = [
        "//cyber/common:cyber_common",
        "//cyber/croutine:cyber_croutine",
        "//cyber/base:cyber_base",
        "//cyber/scheduler:cyber_scheduler",
        "//cyber/time:cyber_time",
    ],
)

apollo_cc_test(
    name = "poller_test",
    size = "small",
    srcs = ["poller_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest",
    ],
)

apollo_cc_binary(
    name = "tcp_echo_client",
    srcs = ["example/tcp_echo_client.cc"],
    deps = [
        "//cyber",
    ],
)

apollo_cc_binary(
    name = "tcp_echo_server",
    srcs = ["example/tcp_echo_server.cc"],
    deps = [
        "//cyber",
    ],
)

apollo_cc_binary(
    name = "udp_echo_client",
    srcs = ["example/udp_echo_client.cc"],
    deps = [
        "//cyber",
    ],
)

apollo_cc_binary(
    name = "udp_echo_server",
    srcs = ["example/udp_echo_server.cc"],
    deps = [
        "//cyber",
    ],
)

apollo_package()
cpplint()
