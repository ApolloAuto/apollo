load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_library", "apollo_cc_test")

package(default_visibility = ["//visibility:public"])

apollo_cc_library(
    name = "cyber_data",
    hdrs = [
        "cache_buffer.h",
        "channel_buffer.h",
        "data_dispatcher.h",
        "data_notifier.h",
        "data_visitor.h",
        "data_visitor_base.h",
        "fusion/all_latest.h",
        "fusion/data_fusion.h",
    ],
    deps = [
        "//cyber/proto:component_conf_cc_proto",
    ],
)

apollo_cc_test(
    name = "cache_buffer_test",
    size = "small",
    srcs = ["cache_buffer_test.cc"],
    deps = [
        ":cyber_data",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "data_visitor_test",
    size = "small",
    srcs = ["data_visitor_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "data_dispatcher_test",
    size = "small",
    srcs = ["data_dispatcher_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "channel_buffer_test",
    size = "small",
    srcs = ["channel_buffer_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_cc_test(
    name = "all_latest_test",
    size = "small",
    srcs = ["fusion/all_latest_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

apollo_package()
cpplint()
