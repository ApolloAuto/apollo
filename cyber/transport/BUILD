load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"],)

cc_library(
    name = "transport",
    srcs = [
        "transport.cc",
        "dispatcher/rtps_dispatcher.cc",
    ],
    hdrs = [
        "transport.h",
        "dispatcher/rtps_dispatcher.h",
    ],
    deps = [
        "history",
        "intra_dispatcher",
        "shm_dispatcher",
        "hybrid_receiver",
        "intra_receiver",
        "rtps_receiver",
        "shm_receiver",
        "hybrid_transmitter",
        "intra_transmitter",
        "rtps_transmitter",
        "shm_transmitter",
        "attributes_filler",
        "sub_listener",
        "underlay_message",
        "underlay_message_type",
        "participant",
        "qos_profile_conf",
        "//cyber/timer:timer_manager",
        "//cyber/service_discovery:role",
        "@fastrtps//:fastrtps",
    ],
)

cc_test(
    name = "transport_test",
    size = "small",
    srcs = [ "transport_test.cc", ],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@gtest",
    ],
)

cc_library(
    name = "endpoint",
    srcs = [ "common/endpoint.cc", ],
    hdrs = [ "common/endpoint.h", ],
    deps = [
        "identity",
        "//cyber/common:global_data",
        "//cyber/proto:role_attributes_cc_proto",
    ],
)

cc_library(
    name = "identity",
    srcs = [ "common/identity.cc", ],
    hdrs = [ "common/identity.h", ],
    deps = [
        "//cyber/common:util",
    ],
)

cc_library(
    name = "syscall_wrapper",
    srcs = [ "common/syscall_wrapper.cc", ],
    hdrs = [ "common/syscall_wrapper.h", ],
    deps = [
        "history",
        "//cyber:binary",
        "//cyber/common:log",
    ],
)

cc_test(
    name = "common_test",
    size = "small",
    srcs = [
        "common/common_test.cc",
    ],
    deps = [
        "//cyber:cyber_core",
        "@gtest//:main",
    ],
)

cc_library(
    name = "dispatcher",
    srcs = [ "dispatcher/dispatcher.cc", ],
    hdrs = [ "dispatcher/dispatcher.h", ],
    deps = [
        "listener_handler",
        "message_info",
        "//cyber/proto:role_attributes_cc_proto",
    ],
)

cc_test(
    name = "dispatcher_test",
    size = "small",
    srcs = [
        "dispatcher/dispatcher_test.cc",
    ],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:chatter_cc_proto",
        "@gtest//:main",
    ],
)

cc_library(
    name = "intra_dispatcher",
    srcs = [ "dispatcher/intra_dispatcher.cc", ],
    hdrs = [ "dispatcher/intra_dispatcher.h", ],
    deps = [
        "//cyber/proto:role_attributes_cc_proto",
        "dispatcher",
    ],
)

cc_test(
    name = "intra_dispatcher_test",
    size = "small",
    srcs = [
        "dispatcher/intra_dispatcher_test.cc",
    ],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:chatter_cc_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "rtps_dispatcher_test",
    size = "small",
    srcs = [
        "dispatcher/rtps_dispatcher_test.cc",
    ],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:chatter_cc_proto",
        "@gtest//:main",
    ],
)

cc_library(
    name = "shm_dispatcher",
    srcs = [ "dispatcher/shm_dispatcher.cc", ],
    hdrs = [ "dispatcher/shm_dispatcher.h", ],
    deps = [
        "dispatcher",
        "syscall_wrapper",
        "segment",
        "readable_info",
        "//cyber/proto:proto_desc_cc_proto",
        "//cyber/message:message_traits",
    ],
)

cc_test(
    name = "shm_dispatcher_test",
    size = "small",
    srcs = [
        "dispatcher/shm_dispatcher_test.cc",
    ],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:chatter_cc_proto",
        "@gtest//:main",
    ],
)

cc_library(
    name = "history_attributes",
    hdrs = [ "message/history_attributes.h", ],
)

cc_library(
    name = "history",
    hdrs = [ "message/history.h", ],
    deps = [
        "history_attributes",
    ],
)

cc_library(
    name = "listener_handler",
    hdrs = [ "message/listener_handler.h", ],
)

cc_library(
    name = "message_info",
    srcs = [ "message/message_info.cc" ],
    hdrs = [ "message/message_info.h" ],
    deps = [
        "//cyber/base:signal",
        "//cyber/common:common",
        "//cyber/message:raw_message",
        "//cyber/transport:identity",
    ],
)

cc_test(
    name = "message_test",
    size = "small",
    srcs = [
        "message/message_test.cc",
    ],
    deps = [
        "//cyber:cyber_core",
        "@gtest//:main",
    ],
)

cc_library(
    name = "qos_profile_conf",
    srcs = [ "qos/qos_profile_conf.cc", ],
    hdrs = [ "qos/qos_profile_conf.h", ],
    deps = [
        "history",
        "//cyber/proto:qos_profile_cc_proto",
    ],
)

cc_library(
    name = "hybrid_receiver",
    hdrs = [ "receiver/hybrid_receiver.h", ],
    deps = [
        "receiver",
    ],
)

cc_library(
    name = "intra_receiver",
    hdrs = [ "receiver/intra_receiver.h", ],
    deps = [
        "receiver",
    ],
)

cc_library(
    name = "receiver",
    hdrs = [ "receiver/receiver.h", ],
    deps = [
        "endpoint",
        "history",
        "message_info",
    ],
)

cc_library(
    name = "rtps_receiver",
    hdrs = [ "receiver/rtps_receiver.h", ],
    deps = [
        "receiver",
    ],
)

cc_library(
    name = "shm_receiver",
    hdrs = [ "receiver/shm_receiver.h", ],
    deps = [
        "receiver",
        "segment",
        "readable_info",
    ],
)

cc_library(
    name = "attributes_filler",
    srcs = [ "rtps/attributes_filler.cc", ],
    hdrs = [ "rtps/attributes_filler.h", ],
    deps = [
        "qos_profile_conf",
        "//cyber/common:log",
        "@fastrtps//:fastrtps",
    ],
)

cc_library(
    name = "underlay_message",
    srcs = [ "rtps/underlay_message.cc", ],
    hdrs = [ "rtps/underlay_message.h", ],
    deps = [
        "@fastrtps//:fastrtps",
    ],
)

cc_library(
    name = "participant",
    srcs = [ "rtps/participant.cc", ],
    hdrs = [ "rtps/participant.h", ],
    deps = [
        "underlay_message",
        "underlay_message_type",
        "//cyber/common:global_data",
    ],
)

cc_library(
    name = "sub_listener",
    srcs = [ "rtps/sub_listener.cc", ],
    hdrs = [ "rtps/sub_listener.h", ],
    deps = [
        "underlay_message_type",
        "underlay_message",
        "message_info"
    ],
)

cc_library(
    name = "underlay_message_type",
    srcs = [ "rtps/underlay_message_type.cc", ],
    hdrs = [ "rtps/underlay_message_type.h", ],
    deps = [
        "underlay_message",
        "@fastrtps//:fastrtps",
    ],
)

cc_test(
    name = "rtps_test",
    size = "small",
    srcs = [ "rtps/rtps_test.cc", ],
    deps = [
        "//cyber:cyber_core",
        "@gtest//:main",
    ],
)

cc_library(
    name = "block",
    srcs = [ "shm/block.cc", ],
    hdrs = [ "shm/block.h", ],
    deps = [
        "//cyber/base:atomic_rw_lock",
        "//cyber/common:log",
    ],
)

cc_library(
    name = "readable_info",
    srcs = [ "shm/readable_info.cc", ],
    hdrs = [ "shm/readable_info.h", ],
    deps = [
        "//cyber/common:log",
    ],
)

cc_library(
    name = "segment",
    srcs = [ "shm/segment.cc", ],
    hdrs = [ "shm/segment.h", ],
    deps = [
        "shm_conf",
        "state",
        "block",
        "//cyber/common:log",
        "//cyber/common:util",
    ],
)

cc_library(
    name = "shm_conf",
    srcs = [ "shm/shm_conf.cc", ],
    hdrs = [ "shm/shm_conf.h", ],
    deps = [
        "//cyber/common:log",
    ],
)

cc_library(
    name = "state",
    srcs = [ "shm/state.cc", ],
    hdrs = [ "shm/state.h", ],
)


cc_library(
    name = "hybrid_transmitter",
    hdrs = [ "transmitter/hybrid_transmitter.h", ],
    deps = [
        "transmitter",
    ],
)

cc_library(
    name = "intra_transmitter",
    hdrs = [ "transmitter/intra_transmitter.h", ],
    deps = [
        "transmitter",
    ],
)

cc_library(
    name = "transmitter",
    hdrs = [ "transmitter/transmitter.h", ],
    deps = [
        "endpoint",
        "message_info",
        "//cyber/event:perf_event_cache",
    ],
)

cc_library(
    name = "rtps_transmitter",
    hdrs = [ "transmitter/rtps_transmitter.h", ],
    deps = [
        "transmitter",
    ],
)

cc_library(
    name = "shm_transmitter",
    hdrs = [ "transmitter/shm_transmitter.h", ],
    deps = [
        "transmitter",
    ],
)

cc_test(
    name = "hybrid_transceiver_test",
    size = "small",
    srcs = [ "transceiver/hybrid_transceiver_test.cc", ],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@gtest",
    ],
)

cc_test(
    name = "intra_transceiver_test",
    size = "small",
    srcs = [ "transceiver/intra_transceiver_test.cc", ],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "rtps_transceiver_test",
    size = "small",
    srcs = [ "transceiver/rtps_transceiver_test.cc", ],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@gtest",
    ],
)

cc_test(
    name = "shm_transceiver_test",
    size = "small",
    srcs = [ "transceiver/shm_transceiver_test.cc", ],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@gtest//:main",
    ],
)

cpplint()
