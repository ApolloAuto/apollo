load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_cc_test", "apollo_cc_binary")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "cyber_node",
    srcs = [
        "node.cc",
    ],
    hdrs = [
        "node.h",
        "node_channel_impl.h",
        "node_service_impl.h",
        "reader.h",
        "reader_base.h",
        "writer.h",
        "writer_base.h",
    ],
    linkopts = ["-lbvar"],
    deps = [
        "//cyber/blocker:cyber_blocker",
        "//cyber/common:cyber_common",
        "//cyber/message:cyber_message",
        "//cyber/proto:run_mode_conf_cc_proto",
        "//cyber/service:cyber_service",
        "//cyber/service_discovery:cyber_service_discovery",
        "//cyber/croutine:cyber_croutine",
        "//cyber/data:cyber_data",
        "//cyber/proto:topology_change_cc_proto",
        "//cyber/scheduler:cyber_scheduler",
        "//cyber/time:cyber_time",
        "//cyber/transport:cyber_transport",
        "//cyber/event:cyber_event",
        "//cyber/proto:role_attributes_cc_proto",
        # "//cyber/statistics:apollo_statistics",
    ],
)

apollo_cc_test(
    name = "node_channel_impl_test",
    size = "small",
    srcs = ["node_channel_impl_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "node_test",
    size = "small",
    srcs = ["node_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "reader_test",
    size = "small",
    srcs = ["reader_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "writer_reader_test",
    size = "small",
    srcs = ["writer_reader_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "writer_test",
    size = "small",
    srcs = ["writer_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest",
    ],
    linkstatic = True,
)

apollo_package()
cpplint()
