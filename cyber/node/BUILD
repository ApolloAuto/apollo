load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "reader_base",
    hdrs = glob([
        "reader_base.h",
    ]),
    deps = [
        "//cyber/transport:transport_lib",
        "//cyber/common:common",
        "//cyber/event:perf_event_cache",
    ],
)

cc_library(
    name = "node",
    srcs = glob([
        "node.cc",
    ]),
    hdrs = glob([
        "*.h",
    ]),
    deps = [
        "//cyber/transport:transport_lib",
        "//cyber/blocker:intra_reader",
        "//cyber/blocker:intra_writer",
        "//cyber/croutine:croutine",
        "//cyber/data:data_visitor",
        "//cyber/scheduler:scheduler",
        "//cyber/service:client",
        "//cyber/service:service",
        "//cyber/service_discovery:topology_manager",
        "//cyber/proto:topology_change_cc_proto",
    ],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = [
        "node_test.cc",
    ],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
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
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@gtest//:main",
    ],
)

cpplint()
