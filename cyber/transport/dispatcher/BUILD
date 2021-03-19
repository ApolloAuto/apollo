load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dispatcher",
    srcs = ["dispatcher.cc"],
    hdrs = ["dispatcher.h"],
    deps = [
        "//cyber/common",
        "//cyber/message:message_traits",
        "//cyber/proto:role_attributes_cc_proto",
        "//cyber/transport/message:listener_handler",
        "//cyber/transport/message:message_info",
    ],
)

cc_test(
    name = "dispatcher_test",
    size = "small",
    srcs = ["dispatcher_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "intra_dispatcher",
    srcs = ["intra_dispatcher.cc"],
    hdrs = ["intra_dispatcher.h"],
    deps = [
        ":dispatcher",
        "//cyber/message:message_traits",
        "//cyber/proto:role_attributes_cc_proto",
    ],
)

cc_test(
    name = "intra_dispatcher_test",
    size = "small",
    srcs = ["intra_dispatcher_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "rtps_dispatcher",
    srcs = ["rtps_dispatcher.cc"],
    hdrs = ["rtps_dispatcher.h"],
    deps = [
        ":dispatcher",
        "//cyber/message:message_traits",
        "//cyber/proto:role_attributes_cc_proto",
        "//cyber/transport/rtps:attributes_filler",
        "//cyber/transport/rtps:participant",
        "//cyber/transport/rtps:sub_listener",
    ],
)

cc_test(
    name = "rtps_dispatcher_test",
    size = "small",
    srcs = ["rtps_dispatcher_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "shm_dispatcher",
    srcs = ["shm_dispatcher.cc"],
    hdrs = ["shm_dispatcher.h"],
    deps = [
        ":dispatcher",
        "//cyber/message:message_traits",
        "//cyber/proto:proto_desc_cc_proto",
        "//cyber/scheduler:scheduler_factory",
        "//cyber/transport/shm:notifier_factory",
        "//cyber/transport/shm:readable_info",
        "//cyber/transport/shm:segment_factory",
    ],
)

cc_test(
    name = "shm_dispatcher_test",
    size = "small",
    srcs = ["shm_dispatcher_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
