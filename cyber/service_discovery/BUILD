load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "topology_manager",
    srcs = ["topology_manager.cc"],
    hdrs = ["topology_manager.h"],
    deps = [
        ":channel_manager",
        ":node_manager",
        ":participant_listener",
        ":service_manager",
        "//cyber/transport:participant",
    ],
)

cc_test(
    name = "topology_manager_test",
    size = "small",
    srcs = ["topology_manager_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "participant_listener",
    srcs = ["communication/participant_listener.cc"],
    hdrs = ["communication/participant_listener.h"],
    deps = [
        "//cyber/common:log",
        "@fastrtps",
    ],
)

cc_library(
    name = "subscriber_listener",
    srcs = ["communication/subscriber_listener.cc"],
    hdrs = ["communication/subscriber_listener.h"],
    deps = [
        "//cyber/common:log",
        "//cyber/transport:underlay_message",
        "//cyber/transport:underlay_message_type",
        "@fastrtps",
    ],
)

cc_library(
    name = "graph",
    srcs = ["container/graph.cc"],
    hdrs = ["container/graph.h"],
    deps = ["//cyber/base:atomic_rw_lock"],
)

cc_test(
    name = "graph_test",
    size = "small",
    srcs = ["container/graph_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "multi_value_warehouse",
    srcs = ["container/multi_value_warehouse.cc"],
    hdrs = ["container/multi_value_warehouse.h"],
    deps = [
        ":warehouse_base",
        "//cyber/base:atomic_rw_lock",
    ],
)

cc_test(
    name = "multi_value_warehouse_test",
    size = "small",
    srcs = ["container/multi_value_warehouse_test.cc"],
    deps = [
        ":multi_value_warehouse",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "single_value_warehouse",
    srcs = ["container/single_value_warehouse.cc"],
    hdrs = ["container/single_value_warehouse.h"],
    deps = [
        ":warehouse_base",
        "//cyber/base:atomic_rw_lock",
    ],
)

cc_test(
    name = "single_value_warehouse_test",
    size = "small",
    srcs = ["container/single_value_warehouse_test.cc"],
    deps = [
        ":single_value_warehouse",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "warehouse_base",
    hdrs = ["container/warehouse_base.h"],
    deps = ["role"],
)

cc_library(
    name = "role",
    srcs = ["role/role.cc"],
    hdrs = ["role/role.h"],
    deps = [
        "//cyber:binary",
        "//cyber/common:log",
        "//cyber/common:types",
        "//cyber/proto:role_attributes_cc_proto",
    ],
)

cc_test(
    name = "role_test",
    size = "small",
    srcs = ["role/role_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "manager",
    srcs = ["specific_manager/manager.cc"],
    hdrs = ["specific_manager/manager.h"],
    deps = [
        ":subscriber_listener",
        "//cyber:state",
        "//cyber/base:signal",
        "//cyber/message:message_traits",
        "//cyber/message:protobuf_factory",
        "//cyber/proto:proto_desc_cc_proto",
        "//cyber/proto:role_attributes_cc_proto",
        "//cyber/proto:topology_change_cc_proto",
        "//cyber/time",
        "//cyber/transport:attributes_filler",
        "//cyber/transport:qos_profile_conf",
        "//cyber/transport:underlay_message_type",
    ],
)

cc_library(
    name = "channel_manager",
    srcs = ["specific_manager/channel_manager.cc"],
    hdrs = ["specific_manager/channel_manager.h"],
    deps = [
        ":graph",
        ":manager",
        ":multi_value_warehouse",
        ":single_value_warehouse",
    ],
)

cc_test(
    name = "channel_manager_test",
    size = "small",
    srcs = ["specific_manager/channel_manager_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "node_manager",
    srcs = ["specific_manager/node_manager.cc"],
    hdrs = ["specific_manager/node_manager.h"],
    deps = [
        ":manager",
        ":multi_value_warehouse",
        ":single_value_warehouse",
    ],
)

cc_test(
    name = "node_manager_test",
    size = "small",
    srcs = ["specific_manager/node_manager_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "service_manager",
    srcs = ["specific_manager/service_manager.cc"],
    hdrs = ["specific_manager/service_manager.h"],
    deps = [
        ":manager",
        ":multi_value_warehouse",
        ":single_value_warehouse",
    ],
)

cc_test(
    name = "service_manager_test",
    size = "small",
    srcs = ["specific_manager/service_manager_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
