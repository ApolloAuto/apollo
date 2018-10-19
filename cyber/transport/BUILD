load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "transport_lib",
    srcs = glob([
        "transport.cc",
        "message/*.cc",
        "receiver/*.cc",
        "transmitter/*.cc",
        "qos/*.cc",
        "rtps/*.cc",
        "shm/*.cc",
        "dispatcher/dispatcher.cc",
        "dispatcher/intra_dispatcher.cc",
        "dispatcher/rtps_dispatcher.cc",
        "dispatcher/shm_dispatcher.cc",
        "reactor/reactor.cc",
    ]),
    hdrs = glob([
        "transport.h",
        "message/*.h",
        "receiver/*.h",
        "transmitter/*.h",
        "qos/*.h",
        "rtps/*.h",
        "shm/*.h",
        "dispatcher/*.h",
        "reactor/*.h",
    ]),
    deps = [
        "//cyber/base:signal",
        "//cyber/common:common",
        "//cyber/message:message_traits",
        "//cyber/message:raw_message",
        "//cyber/proto:qos_profile_cc_proto",
        "//cyber/proto:role_attributes_cc_proto",
        "//cyber/proto:transport_conf_cc_proto",
        "//cyber/service_discovery:role",
        "//cyber/transport/rtps:rtps_lib",
        "//cyber/transport/message:transport_message_lib",
        "//cyber/transport/common:transport_common",
        "//cyber/time:time",
        "//cyber/timer:timer_manager",
    ],
)

cc_test(
    name = "dispatcher_test",
    size = "small",
    srcs = [
        "dispatcher_test.cc",
    ],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:chatter_cc_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "hybrid_transceiver_test",
    size = "small",
    srcs = [
        "hybrid_transceiver_test.cc",
    ],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@gtest",
    ],
)

cc_test(
    name = "iintra_dispatcher_test",
    size = "small",
    srcs = [
        "intra_dispatcher_test.cc",
    ],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:chatter_cc_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "intra_transceiver_test",
    size = "small",
    srcs = [
        "intra_transceiver_test.cc",
    ],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "imessage_test",
    size = "small",
    srcs = [
        "message_test.cc",
    ],
    deps = [
        "//cyber:cyber_core",
        "@gtest//:main",
    ],
)

cc_test(
    name = "reactor_test",
    size = "small",
    srcs = [ "reactor/reactor_test.cc", ],
    deps = [
        "//cyber:cyber_core",
        "@gtest//:main",
    ],
)

cc_test(
    name = "rtps_dispatcher_test",
    size = "small",
    srcs = [
        "rtps_dispatcher_test.cc",
    ],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:chatter_cc_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "rtps_test",
    size = "small",
    srcs = [
        "rtps_test.cc",
    ],
    deps = [
        "//cyber:cyber_core",
        "@gtest//:main",
    ],
)

cc_test(
    name = "rtps_transceiver_test",
    size = "small",
    srcs = [
        "rtps_transceiver_test.cc",
    ],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@gtest",
    ],
)

cc_test(
    name = "shm_dispatcher_test",
    size = "small",
    srcs = [
        "shm_dispatcher_test.cc",
    ],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:chatter_cc_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "shm_transceiver_test",
    size = "small",
    srcs = [
        "shm_transceiver_test.cc",
    ],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "transport_test",
    size = "small",
    srcs = [
        "transport_test.cc",
    ],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@gtest",
    ],
)

cpplint()
