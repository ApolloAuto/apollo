load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "node",
    srcs = ["node.cc"],
    hdrs = ["node.h"],
    deps = [
        ":node_channel_impl",
        ":node_service_impl",
    ],
)

cc_library(
    name = "node_channel_impl",
    hdrs = ["node_channel_impl.h"],
    deps = [
        ":reader",
        ":writer",
        "//cyber/blocker:intra_reader",
        "//cyber/blocker:intra_writer",
        "//cyber/common:global_data",
        "//cyber/message:message_traits",
        "//cyber/proto:run_mode_conf_cc_proto",
    ],
)

cc_test(
    name = "node_channel_impl_test",
    size = "small",
    srcs = ["node_channel_impl_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "node_service_impl",
    hdrs = ["node_service_impl.h"],
    deps = [
        ":reader",
        ":writer",
        "//cyber/common:global_data",
        "//cyber/service",
        "//cyber/service:client",
        "//cyber/service_discovery:topology_manager",
    ],
)

cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "reader",
    hdrs = ["reader.h"],
    deps = [
        ":reader_base",
        "//cyber/blocker",
        "//cyber/common:global_data",
        "//cyber/croutine:routine_factory",
        "//cyber/data:data_visitor",
        "//cyber/proto:topology_change_cc_proto",
        "//cyber/scheduler",
        "//cyber/service_discovery:topology_manager",
        "//cyber/time",
        "//cyber/transport",
    ],
)

cc_library(
    name = "reader_base",
    hdrs = ["reader_base.h"],
    deps = [
        "//cyber/event:perf_event_cache",
        "//cyber/transport",
    ],
)

cc_test(
    name = "reader_test",
    size = "small",
    srcs = ["reader_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "writer",
    hdrs = ["writer.h"],
    deps = [
        ":writer_base",
        "//cyber/common:log",
        "//cyber/proto:topology_change_cc_proto",
        "//cyber/service_discovery:topology_manager",
        "//cyber/transport",
    ],
)

cc_library(
    name = "writer_base",
    hdrs = ["writer_base.h"],
    deps = [
        "//cyber/common:macros",
        "//cyber/common:util",
        "//cyber/proto:role_attributes_cc_proto",
    ],
)

cc_test(
    name = "writer_reader_test",
    size = "small",
    srcs = ["writer_reader_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "writer_test",
    size = "small",
    srcs = ["writer_test.cc"],
    deps = [
        "//cyber",
        "//cyber/node:writer_base",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cpplint()
