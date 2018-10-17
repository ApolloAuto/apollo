load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "service_discovery_lib",
    srcs = glob([
        "topology_manager.cc",
        "container/*.cc",
        "specific_manager/*.cc",
        "communication/*.cc",
        "role/*.cc",
    ]),
    hdrs = glob([
        "*.h",
        "**/*.h",
    ]),
    deps = [
        "//cybertron:init",
        "//cybertron/base:signal",
        "//cybertron/common:common",
        "//cybertron/message:message_traits",
        "//cybertron/message:protobuf_factory",
        "//cybertron/proto:role_attributes_cc_proto",
        "//cybertron/proto:topology_change_cc_proto",
        "//cybertron/time:time",
        "//cybertron/transport/rtps:rtps_lib",
        "//cybertron/transport/qos:qos_lib",
    ],
)

cc_test(
    name = "channel_manager_test",
    size = "small",
    srcs = [
        "channel_manager_test.cc",
    ],
    deps = [
        "//cybertron",
        "//cybertron/proto:chatter_cc_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "graph_test",
    size = "small",
    srcs = [
        "graph_test.cc",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_test(
    name = "node_manager_test",
    size = "small",
    srcs = [
        "node_manager_test.cc",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_test(
    name = "role_test",
    size = "small",
    srcs = [
        "role_test.cc",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_test(
    name = "service_manager_test",
    size = "small",
    srcs = [
        "service_manager_test.cc",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_test(
    name = "topology_manager_test",
    size = "small",
    srcs = [
        "topology_manager_test.cc",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cpplint()
