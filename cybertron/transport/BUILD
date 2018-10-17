load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "transport_lib",
    srcs = glob([
        "transport.cc",
        "message/*.cc",
        "common/*.cc",
        "receiver/*.cc",
        "transmitter/*.cc",
        "qos/*.cc",
        "rtps/*.cc",
        "shm/*.cc",
        "dispatcher/*.cc",
        "reactor/*.cc",
    ]),
    hdrs = glob([
        "*.h",
        "**/*.h",
    ]),
    deps = [
        "//cybertron/common:common",
        "//cybertron/message:message_traits",
        "//cybertron/message:raw_message",
        "//cybertron/proto:qos_profile_cc_proto",
        "//cybertron/proto:role_attributes_cc_proto",
        "//cybertron/proto:transport_conf_cc_proto",
        "//cybertron/service_discovery:service_discovery_lib",
        "//cybertron/transport/rtps:rtps_lib",
        "//cybertron/transport/common:transport_common",
        "//cybertron/transport/message:transport_message_lib",
        "//cybertron/time:time",
        "//cybertron/timer:timer_lib",
    ],
)

cc_test(
    name = "common_test",
    size = "small",
    srcs = [
        "common_test.cc",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_test(
    name = "dispatcher_test",
    size = "small",
    srcs = [
        "dispatcher_test.cc",
    ],
    deps = [
        "//cybertron",
        "//cybertron/proto:chatter_cc_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "hybrid_transceiver_test",
    size = "small",
    srcs = [
        "hybrid_transceiver_test.cc",
    ],
    deps = [
        "//cybertron",
        "//cybertron/proto:unit_test_cc_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "iintra_dispatcher_test",
    size = "small",
    srcs = [
        "intra_dispatcher_test.cc",
    ],
    deps = [
        "//cybertron",
        "//cybertron/proto:chatter_cc_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "intra_transceiver_test",
    size = "small",
    srcs = [
        "intra_transceiver_test.cc",
    ],
    deps = [
        "//cybertron",
        "//cybertron/proto:unit_test_cc_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "imessage_test",
    size = "small",
    srcs = [
        "message_test.cc",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_test(
    name = "reactor_test",
    size = "small",
    srcs = [
        "reactor_test.cc",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_test(
    name = "rtps_dispatcher_test",
    size = "small",
    srcs = [
        "rtps_dispatcher_test.cc",
    ],
    deps = [
        "//cybertron",
        "//cybertron/proto:chatter_cc_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "rtps_test",
    size = "small",
    srcs = [
        "rtps_test.cc",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_test(
    name = "rtps_transceiver_test",
    size = "small",
    srcs = [
        "rtps_transceiver_test.cc",
    ],
    deps = [
        "//cybertron",
        "//cybertron/proto:unit_test_cc_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "shm_dispatcher_test",
    size = "small",
    srcs = [
        "shm_dispatcher_test.cc",
    ],
    deps = [
        "//cybertron",
        "//cybertron/proto:chatter_cc_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "shm_transceiver_test",
    size = "small",
    srcs = [
        "shm_transceiver_test.cc",
    ],
    deps = [
        "//cybertron",
        "//cybertron/proto:unit_test_cc_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "transport_test",
    size = "small",
    srcs = [
        "transport_test.cc",
    ],
    deps = [
        "//cybertron",
        "//cybertron/proto:unit_test_cc_proto",
        "@gtest//:main",
    ],
)

cpplint()
