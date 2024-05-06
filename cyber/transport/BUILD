load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package", "apollo_cc_test")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "cyber_transport",
    srcs = [
        'transport.cc', 'shm/segment.cc', 'shm/condition_notifier.cc', 
        'shm/segment_factory.cc', 'shm/posix_segment.cc', 'shm/state.cc', 
        'shm/multicast_notifier.cc', 'shm/block.cc', 'shm/shm_conf.cc', 
        'shm/xsi_segment.cc', 'shm/readable_info.cc', 'shm/notifier_factory.cc', 
        'qos/qos_profile_conf.cc', 'common/identity.cc', 'common/endpoint.cc', 
        'dispatcher/intra_dispatcher.cc', 'dispatcher/shm_dispatcher.cc', 
        'dispatcher/rtps_dispatcher.cc', 'dispatcher/dispatcher.cc', 
        'message/message_info.cc', 'rtps/participant.cc', 'rtps/attributes_filler.cc', 
        'rtps/sub_listener.cc', 'rtps/underlay_message_type.cc', 
        'rtps/underlay_message.cc'
    ],
    hdrs = [
        'transport.h', 'shm/state.h', 'shm/xsi_segment.h', 
        'shm/notifier_factory.h', 'shm/block.h', 'shm/shm_conf.h', 
        'shm/readable_info.h', 'shm/posix_segment.h', 'shm/segment_factory.h', 
        'shm/multicast_notifier.h', 'shm/segment.h', 'shm/notifier_base.h', 
        'shm/condition_notifier.h', 'qos/qos_profile_conf.h', 'common/identity.h', 
        'common/endpoint.h', 'receiver/hybrid_receiver.h', 'receiver/shm_receiver.h', 
        'receiver/receiver.h', 'receiver/intra_receiver.h', 'receiver/rtps_receiver.h', 
        'transmitter/rtps_transmitter.h', 'transmitter/transmitter.h', 
        'transmitter/shm_transmitter.h', 'transmitter/hybrid_transmitter.h', 
        'transmitter/intra_transmitter.h', 'dispatcher/dispatcher.h', 
        'dispatcher/intra_dispatcher.h', 'dispatcher/rtps_dispatcher.h', 
        'dispatcher/shm_dispatcher.h', 'message/history.h', 'message/listener_handler.h', 
        'message/history_attributes.h', 'message/message_info.h', 
        'rtps/attributes_filler.h', 'rtps/underlay_message.h', 'rtps/participant.h', 
        'rtps/sub_listener.h', 'rtps/underlay_message_type.h'
    ],
    linkopts = ["-lrt"],
    deps = [
        "@fastrtps",
        "@uuid",
        "//cyber/service_discovery:cyber_service_discovery_role",
        "//cyber/task:cyber_task",
        "//cyber/common:cyber_common",
        "//cyber/proto:role_attributes_cc_proto",
        "//cyber/message:cyber_message",
        "//cyber/proto:proto_desc_cc_proto",
        "//cyber/scheduler:cyber_scheduler",
        "//cyber/proto:qos_profile_cc_proto",
        "//cyber/base:cyber_base",
        "//cyber/event:cyber_event",
    ],
)

apollo_cc_test(
    name = "condition_notifier_test",
    size = "small",
    srcs = ["shm/condition_notifier_test.cc"],
    tags = ["exclusive"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
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
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
)

apollo_cc_test(
    name = "message_test",
    size = "small",
    srcs = ["message/message_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
    linkstatic = True,
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
