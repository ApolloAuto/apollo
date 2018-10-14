load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "transport_lib",
    srcs = glob([
        "transport.cpp",
        "message/*.cpp",
        "common/*.cpp",
        "receiver/*.cpp",
        "transmitter/*.cpp",
        "qos/*.cpp",
        "rtps/*.cpp",
        "shm/*.cpp",
        "dispatcher/*.cpp",
        "reactor/*.cpp",
    ]),
    hdrs = glob([
        "*.h",
        "**/*.h",
    ]),
    deps = [
        "//cybertron/base:base_lib",
        "//cybertron/common:common_lib",
        "//cybertron/event:event_lib",
        "//cybertron/message:message_lib",
        "//cybertron/proto:qos_profile_cc_proto",
        "//cybertron/proto:role_attributes_cc_proto",
        "//cybertron/proto:transport_conf_cc_proto",
        "//cybertron/service_discovery:service_discovery_lib",
        "//cybertron/transport/rtps:rtps_lib",
        "//cybertron/transport/common:transport_common_lib",
        "//cybertron/transport/message:transport_message_lib",
        "//cybertron/time:time_lib",
        "//cybertron/timer:timer_lib",
    ],
)

cc_test(
    name = "common_test",
    size = "small",
    srcs = [
        "common_test.cpp",
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
        "dispatcher_test.cpp",
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
        "hybrid_transceiver_test.cpp",
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
        "intra_dispatcher_test.cpp",
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
        "intra_transceiver_test.cpp",
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
        "message_test.cpp",
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
        "reactor_test.cpp",
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
        "rtps_dispatcher_test.cpp",
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
        "rtps_test.cpp",
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
        "rtps_transceiver_test.cpp",
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
        "shm_dispatcher_test.cpp",
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
        "shm_transceiver_test.cpp",
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
        "transport_test.cpp",
    ],
    deps = [
        "//cybertron",
        "//cybertron/proto:unit_test_cc_proto",
        "@gtest//:main",
    ],
)

cpplint()
