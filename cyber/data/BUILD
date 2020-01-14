load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "data",
    deps = [
        ":all_latest",
        ":cache_buffer",
        ":channel_buffer",
        ":data_dispatcher",
        ":data_fusion",
        ":data_notifier",
        ":data_visitor",
        ":data_visitor_base",
    ],
)

cc_library(
    name = "cache_buffer",
    srcs = ["cache_buffer.h"],
)

cc_test(
    name = "cache_buffer_test",
    size = "small",
    srcs = ["cache_buffer_test.cc"],
    deps = [
        ":cache_buffer",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "channel_buffer",
    hdrs = ["channel_buffer.h"],
    deps = [
        ":data_notifier",
        "//cyber/proto:component_conf_cc_proto",
    ],
)

cc_library(
    name = "data_dispatcher",
    hdrs = ["data_dispatcher.h"],
    deps = [
        ":channel_buffer",
    ],
)

cc_library(
    name = "data_visitor",
    hdrs = ["data_visitor.h"],
)

cc_test(
    name = "data_visitor_test",
    size = "small",
    srcs = ["data_visitor_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "data_visitor_base",
    hdrs = ["data_visitor_base.h"],
)

cc_library(
    name = "data_notifier",
    hdrs = ["data_notifier.h"],
    deps = [
        ":cache_buffer",
    ],
)

cc_test(
    name = "data_dispatcher_test",
    size = "small",
    srcs = ["data_dispatcher_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "channel_buffer_test",
    size = "small",
    srcs = ["channel_buffer_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "data_fusion",
    hdrs = ["fusion/data_fusion.h"],
)

cc_library(
    name = "all_latest",
    hdrs = ["fusion/all_latest.h"],
    deps = [
        ":channel_buffer",
        ":data_fusion",
    ],
)

cc_test(
    name = "all_latest_test",
    size = "small",
    srcs = ["fusion/all_latest_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
