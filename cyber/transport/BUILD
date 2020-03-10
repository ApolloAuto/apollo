load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "transport",
    srcs = ["transport.cc"],
    hdrs = ["transport.h"],
    deps = [
        ":attributes_filler",
        ":history",
        ":hybrid_receiver",
        ":hybrid_transmitter",
        ":intra_dispatcher",
        ":intra_receiver",
        ":intra_transmitter",
        ":participant",
        ":qos_profile_conf",
        ":rtps_dispatcher",
        ":rtps_receiver",
        ":rtps_transmitter",
        ":shm_dispatcher",
        ":shm_receiver",
        ":shm_transmitter",
        ":sub_listener",
        ":underlay_message",
        ":underlay_message_type",
        "//cyber/service_discovery:role",
        "//cyber/task",
        "@fastrtps",
    ],
)

cc_test(
    name = "transport_test",
    size = "small",
    srcs = ["transport_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "endpoint",
    srcs = ["common/endpoint.cc"],
    hdrs = ["common/endpoint.h"],
    deps = [
        ":identity",
        "//cyber/common:global_data",
        "//cyber/proto:role_attributes_cc_proto",
    ],
)

cc_test(
    name = "endpoint_test",
    size = "small",
    srcs = ["common/endpoint_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "identity",
    srcs = ["common/identity.cc"],
    hdrs = ["common/identity.h"],
    deps = [
        "//cyber/common:util",
    ],
)

cc_test(
    name = "identity_test",
    size = "small",
    srcs = ["common/identity_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "common_test",
    size = "small",
    srcs = ["common/common_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "dispatcher",
    srcs = ["dispatcher/dispatcher.cc"],
    hdrs = ["dispatcher/dispatcher.h"],
    deps = [
        ":listener_handler",
        ":message_info",
        "//cyber/message:message_traits",
        "//cyber/proto:role_attributes_cc_proto",
    ],
)

cc_test(
    name = "dispatcher_test",
    size = "small",
    srcs = ["dispatcher/dispatcher_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "intra_dispatcher",
    srcs = ["dispatcher/intra_dispatcher.cc"],
    hdrs = ["dispatcher/intra_dispatcher.h"],
    deps = [
        ":dispatcher",
        "//cyber/message:message_traits",
        "//cyber/proto:role_attributes_cc_proto",
    ],
)

cc_test(
    name = "intra_dispatcher_test",
    size = "small",
    srcs = ["dispatcher/intra_dispatcher_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "rtps_dispatcher",
    srcs = ["dispatcher/rtps_dispatcher.cc"],
    hdrs = ["dispatcher/rtps_dispatcher.h"],
    deps = [
        ":attributes_filler",
        ":dispatcher",
        ":participant",
        ":sub_listener",
        "//cyber/message:message_traits",
        "//cyber/proto:role_attributes_cc_proto",
    ],
)

cc_test(
    name = "rtps_dispatcher_test",
    size = "small",
    srcs = ["dispatcher/rtps_dispatcher_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "shm_dispatcher",
    srcs = ["dispatcher/shm_dispatcher.cc"],
    hdrs = ["dispatcher/shm_dispatcher.h"],
    deps = [
        ":dispatcher",
        ":notifier_factory",
        ":readable_info",
        ":segment_factory",
        "//cyber/message:message_traits",
        "//cyber/proto:proto_desc_cc_proto",
        "//cyber/scheduler:scheduler_factory",
    ],
)

cc_test(
    name = "shm_dispatcher_test",
    size = "small",
    srcs = ["dispatcher/shm_dispatcher_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "history_attributes",
    hdrs = ["message/history_attributes.h"],
)

cc_library(
    name = "history",
    hdrs = ["message/history.h"],
    deps = [
        ":history_attributes",
    ],
)

cc_library(
    name = "listener_handler",
    hdrs = ["message/listener_handler.h"],
)

cc_library(
    name = "message_info",
    srcs = ["message/message_info.cc"],
    hdrs = ["message/message_info.h"],
    deps = [
        "//cyber/base:signal",
        "//cyber/common",
        "//cyber/message:raw_message",
        "//cyber/transport:identity",
    ],
)

cc_test(
    name = "message_info_test",
    size = "small",
    srcs = ["message/message_info_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "message_test",
    size = "small",
    srcs = ["message/message_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "qos_profile_conf",
    srcs = ["qos/qos_profile_conf.cc"],
    hdrs = ["qos/qos_profile_conf.h"],
    deps = [
        ":history",
        "//cyber/proto:qos_profile_cc_proto",
    ],
)

cc_library(
    name = "hybrid_receiver",
    hdrs = ["receiver/hybrid_receiver.h"],
    deps = [
        ":receiver",
    ],
)

cc_library(
    name = "intra_receiver",
    hdrs = ["receiver/intra_receiver.h"],
    deps = [
        ":receiver",
    ],
)

cc_library(
    name = "receiver",
    hdrs = ["receiver/receiver.h"],
    deps = [
        ":endpoint",
        ":history",
        ":message_info",
    ],
)

cc_library(
    name = "rtps_receiver",
    hdrs = ["receiver/rtps_receiver.h"],
    deps = [
        ":receiver",
    ],
)

cc_library(
    name = "shm_receiver",
    hdrs = ["receiver/shm_receiver.h"],
    deps = [
        ":readable_info",
        ":receiver",
    ],
)

cc_library(
    name = "attributes_filler",
    srcs = ["rtps/attributes_filler.cc"],
    hdrs = ["rtps/attributes_filler.h"],
    deps = [
        ":qos_profile_conf",
        "//cyber/common:log",
        "@fastrtps",
    ],
)

cc_library(
    name = "underlay_message",
    srcs = ["rtps/underlay_message.cc"],
    hdrs = ["rtps/underlay_message.h"],
    deps = [
        "//cyber/common:log",
        "@fastrtps",
    ],
)

cc_library(
    name = "participant",
    srcs = ["rtps/participant.cc"],
    hdrs = ["rtps/participant.h"],
    deps = [
        ":underlay_message",
        ":underlay_message_type",
        "//cyber/common:global_data",
    ],
)

cc_library(
    name = "sub_listener",
    srcs = ["rtps/sub_listener.cc"],
    hdrs = ["rtps/sub_listener.h"],
    deps = [
        ":message_info",
        ":underlay_message",
        ":underlay_message_type",
    ],
)

cc_library(
    name = "underlay_message_type",
    srcs = ["rtps/underlay_message_type.cc"],
    hdrs = ["rtps/underlay_message_type.h"],
    deps = [
        ":underlay_message",
        "@fastrtps",
    ],
)

cc_test(
    name = "rtps_test",
    size = "small",
    srcs = ["rtps/rtps_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "block",
    srcs = ["shm/block.cc"],
    hdrs = ["shm/block.h"],
    deps = [
        "//cyber/base:atomic_rw_lock",
        "//cyber/common:log",
    ],
)

cc_library(
    name = "condition_notifier",
    srcs = ["shm/condition_notifier.cc"],
    hdrs = ["shm/condition_notifier.h"],
    deps = [
        ":notifier_base",
        "//cyber/common:global_data",
        "//cyber/common:log",
        "//cyber/common:util",
    ],
)

cc_library(
    name = "multicast_notifier",
    srcs = ["shm/multicast_notifier.cc"],
    hdrs = ["shm/multicast_notifier.h"],
    deps = [
        ":notifier_base",
        "//cyber/common:global_data",
        "//cyber/common:log",
        "//cyber/common:macros",
    ],
)

cc_library(
    name = "notifier_base",
    hdrs = ["shm/notifier_base.h"],
    deps = [
        ":readable_info",
    ],
)

cc_library(
    name = "notifier_factory",
    srcs = ["shm/notifier_factory.cc"],
    hdrs = ["shm/notifier_factory.h"],
    deps = [
        ":condition_notifier",
        ":multicast_notifier",
        ":notifier_base",
        "//cyber/common:global_data",
        "//cyber/common:log",
    ],
)

cc_library(
    name = "readable_info",
    srcs = ["shm/readable_info.cc"],
    hdrs = ["shm/readable_info.h"],
    deps = [
        "//cyber/common:log",
    ],
)

cc_library(
    name = "xsi_segment",
    srcs = ["shm/xsi_segment.cc"],
    hdrs = ["shm/xsi_segment.h"],
    deps = [
        ":segment",
        "//cyber/common:log",
        "//cyber/common:util",
    ],
)

cc_library(
    name = "posix_segment",
    srcs = ["shm/posix_segment.cc"],
    hdrs = ["shm/posix_segment.h"],
    deps = [
        ":segment",
        "//cyber/common:log",
        "//cyber/common:util",
    ],
)

cc_library(
    name = "segment",
    srcs = ["shm/segment.cc"],
    hdrs = ["shm/segment.h"],
    deps = [
        ":block",
        ":shm_conf",
        ":state",
        "//cyber/common:log",
        "//cyber/common:util",
    ],
)

cc_library(
    name = "segment_factory",
    srcs = ["shm/segment_factory.cc"],
    hdrs = ["shm/segment_factory.h"],
    deps = [
        ":posix_segment",
        ":segment",
        ":xsi_segment",
        "//cyber/common:global_data",
        "//cyber/common:log",
    ],
)

cc_library(
    name = "shm_conf",
    srcs = ["shm/shm_conf.cc"],
    hdrs = ["shm/shm_conf.h"],
    deps = [
        "//cyber/common:log",
    ],
)

cc_library(
    name = "state",
    srcs = ["shm/state.cc"],
    hdrs = ["shm/state.h"],
)

cc_library(
    name = "hybrid_transmitter",
    hdrs = ["transmitter/hybrid_transmitter.h"],
    deps = [
        ":transmitter",
    ],
)

cc_library(
    name = "intra_transmitter",
    hdrs = ["transmitter/intra_transmitter.h"],
    deps = [
        ":transmitter",
    ],
)

cc_library(
    name = "transmitter",
    hdrs = ["transmitter/transmitter.h"],
    deps = [
        ":endpoint",
        ":message_info",
        "//cyber/event:perf_event_cache",
    ],
)

cc_library(
    name = "rtps_transmitter",
    hdrs = ["transmitter/rtps_transmitter.h"],
    deps = [
        ":transmitter",
    ],
)

cc_library(
    name = "shm_transmitter",
    hdrs = ["transmitter/shm_transmitter.h"],
    deps = [
        ":transmitter",
    ],
)

cc_test(
    name = "hybrid_transceiver_test",
    size = "small",
    srcs = ["transceiver/hybrid_transceiver_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "intra_transceiver_test",
    size = "small",
    srcs = ["transceiver/intra_transceiver_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "rtps_transceiver_test",
    size = "small",
    srcs = ["transceiver/rtps_transceiver_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "shm_transceiver_test",
    size = "small",
    srcs = ["transceiver/shm_transceiver_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "condition_notifier_test",
    size = "small",
    srcs = ["shm/condition_notifier_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
