load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_cc_test", "apollo_package")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "cyber_transport",
    srcs = [
        "common/endpoint.cc",
        "common/identity.cc",
        "dispatcher/dispatcher.cc",
        "dispatcher/intra_dispatcher.cc",
        "dispatcher/rtps_dispatcher.cc",
        "dispatcher/shm_dispatcher.cc",
        "message/message_info.cc",
        "qos/qos_profile_conf.cc",
        "rtps/attributes_filler.cc",
        "rtps/participant.cc",
        "rtps/sub_listener.cc",
        "rtps/underlay_message.cc",
        "rtps/underlay_message_type.cc",
        "shm/arena_address_allocator.cc",
        "shm/block.cc",
        "shm/condition_notifier.cc",
        "shm/multicast_notifier.cc",
        "shm/notifier_factory.cc",
        "shm/posix_segment.cc",
        "shm/protobuf_arena_manager.cc",
        "shm/readable_info.cc",
        "shm/segment.cc",
        "shm/segment_factory.cc",
        "shm/shm_conf.cc",
        "shm/state.cc",
        "shm/xsi_segment.cc",
        "transport.cc",
    ],
    hdrs = [
        "common/endpoint.h",
        "common/identity.h",
        "dispatcher/dispatcher.h",
        "dispatcher/intra_dispatcher.h",
        "dispatcher/rtps_dispatcher.h",
        "dispatcher/shm_dispatcher.h",
        "message/history.h",
        "message/history_attributes.h",
        "message/listener_handler.h",
        "message/message_info.h",
        "qos/qos_profile_conf.h",
        "receiver/hybrid_receiver.h",
        "receiver/intra_receiver.h",
        "receiver/receiver.h",
        "receiver/rtps_receiver.h",
        "receiver/shm_receiver.h",
        "rtps/attributes_filler.h",
        "rtps/participant.h",
        "rtps/sub_listener.h",
        "rtps/underlay_message.h",
        "rtps/underlay_message_type.h",
        "shm/arena_address_allocator.h",
        "shm/block.h",
        "shm/condition_notifier.h",
        "shm/multicast_notifier.h",
        "shm/notifier_base.h",
        "shm/notifier_factory.h",
        "shm/posix_segment.h",
        "shm/protobuf_arena_manager.h",
        "shm/readable_info.h",
        "shm/segment.h",
        "shm/segment_factory.h",
        "shm/shm_conf.h",
        "shm/state.h",
        "shm/xsi_segment.h",
        "transmitter/hybrid_transmitter.h",
        "transmitter/intra_transmitter.h",
        "transmitter/rtps_transmitter.h",
        "transmitter/shm_transmitter.h",
        "transmitter/transmitter.h",
        "transport.h",
    ],
    linkopts = ["-lrt"],
    deps = [
        "//cyber/base:cyber_base",
        "//cyber/common:cyber_common",
        "//cyber/event:cyber_event",
        "//cyber/message:cyber_message",
        "//cyber/proto:proto_desc_cc_proto",
        "//cyber/proto:qos_profile_cc_proto",
        "//cyber/proto:role_attributes_cc_proto",
        "//cyber/scheduler:cyber_scheduler",
        "//cyber/service_discovery:cyber_service_discovery_role",
        "//cyber/statistics:apollo_statistics",
        "//cyber/task:cyber_task",
        "@fastrtps",
        "@uuid",
    ],
)

apollo_cc_test(
    name = "condition_notifier_test",
    size = "small",
    srcs = ["shm/condition_notifier_test.cc"],
    linkstatic = True,
    tags = ["exclusive"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "rtps_test",
    size = "small",
    srcs = ["rtps/rtps_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
        "@fastrtps",
    ],
)

apollo_cc_test(
    name = "message_info_test",
    size = "small",
    srcs = ["message/message_info_test.cc"],
    linkstatic = True,
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "message_test",
    size = "small",
    srcs = ["message/message_test.cc"],
    linkstatic = True,
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "endpoint_test",
    size = "small",
    srcs = ["common/endpoint_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "identity_test",
    size = "small",
    srcs = ["common/identity_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "transport_test",
    size = "small",
    srcs = ["transport_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

apollo_cc_test(
    name = "dispatcher_test",
    size = "small",
    srcs = ["dispatcher/dispatcher_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "intra_dispatcher_test",
    size = "small",
    srcs = ["dispatcher/intra_dispatcher_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "rtps_dispatcher_test",
    size = "small",
    srcs = ["dispatcher/rtps_dispatcher_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

apollo_cc_test(
    name = "shm_dispatcher_test",
    size = "small",
    srcs = ["dispatcher/shm_dispatcher_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()

cpplint()
