load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "reader_base",
    hdrs = glob([
        "reader_base.h",
    ]),
    deps = [
        "//cybertron/transport:transport_lib",
        "//cybertron/common:common",
        "//cybertron/event:perf_event_cache",
    ],
)

cc_library(
    name = "node_lib",
    srcs = glob([
        "node.cc",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//cybertron/transport:transport_lib",
        "//cybertron/blocker:intra_reader",
        "//cybertron/blocker:intra_writer",
        "//cybertron/croutine:croutine",
        "//cybertron/data:data_visitor",
        "//cybertron/scheduler:scheduler",
        "//cybertron/service:client",
        "//cybertron/service:service",
        "//cybertron/service_discovery:topology_manager",
        "//cybertron/proto:topology_change_cc_proto",
    ],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = [
        "node_test.cc",
    ],
    deps = [
        "//cybertron",
        "//cybertron/proto:unit_test_cc_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "writer_reader_test",
    size = "small",
    srcs = [
        "writer_reader_test.cc",
    ],
    deps = [
        "//cybertron",
        "//cybertron/proto:unit_test_cc_proto",
        "@gtest//:main",
    ],
)

cpplint()
