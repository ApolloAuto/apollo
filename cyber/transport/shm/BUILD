load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "block",
    srcs = ["block.cc"],
    hdrs = ["block.h"],
    deps = [
        "//cyber/base:atomic_rw_lock",
        "//cyber/common:log",
    ],
)

cc_library(
    name = "condition_notifier",
    srcs = ["condition_notifier.cc"],
    hdrs = ["condition_notifier.h"],
    deps = [
        ":notifier_base",
        "//cyber/common:global_data",
        "//cyber/common:log",
        "//cyber/common:util",
    ],
)

cc_library(
    name = "multicast_notifier",
    srcs = ["multicast_notifier.cc"],
    hdrs = ["multicast_notifier.h"],
    deps = [
        ":notifier_base",
        "//cyber/common:global_data",
        "//cyber/common:log",
        "//cyber/common:macros",
    ],
)

cc_library(
    name = "notifier_base",
    hdrs = ["notifier_base.h"],
    deps = [
        ":readable_info",
    ],
)

cc_library(
    name = "notifier_factory",
    srcs = ["notifier_factory.cc"],
    hdrs = ["notifier_factory.h"],
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
    srcs = ["readable_info.cc"],
    hdrs = ["readable_info.h"],
    deps = [
        "//cyber/common:log",
    ],
)

cc_library(
    name = "xsi_segment",
    srcs = ["xsi_segment.cc"],
    hdrs = ["xsi_segment.h"],
    deps = [
        ":segment",
        "//cyber/common:log",
        "//cyber/common:util",
    ],
)

cc_library(
    name = "posix_segment",
    srcs = ["posix_segment.cc"],
    hdrs = ["posix_segment.h"],
    deps = [
        ":segment",
        "//cyber/common:log",
        "//cyber/common:util",
    ],
)

cc_library(
    name = "segment",
    srcs = ["segment.cc"],
    hdrs = ["segment.h"],
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
    srcs = ["segment_factory.cc"],
    hdrs = ["segment_factory.h"],
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
    srcs = ["shm_conf.cc"],
    hdrs = ["shm_conf.h"],
    deps = [
        "//cyber/common:log",
    ],
)

cc_library(
    name = "state",
    srcs = ["state.cc"],
    hdrs = ["state.h"],
)

cc_test(
    name = "condition_notifier_test",
    size = "small",
    srcs = ["condition_notifier_test.cc"],
    deps = [
        "//cyber:cyber_core",
        "@com_google_googletest//:gtest_main",
    ],
)

cpplint()
