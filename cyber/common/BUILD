load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    deps = [
        "//cyber/common:environment",
        "//cyber/common:file",
        "//cyber/common:global_data",
        "//cyber/common:log",
        "//cyber/common:macros",
        "//cyber/common:time_conversion",
        "//cyber/common:types",
        "//cyber/common:util",
    ],
)

cc_library(
    name = "file",
    srcs = ["file.cc"],
    hdrs = ["file.h"],
    deps = [
        "//cyber/common:log",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "file_test",
    size = "small",
    srcs = ["file_test.cc"],
    deps = [
        "//cyber",
        "//cyber/proto:unit_test_cc_proto",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "log",
    hdrs = ["log.h"],
    deps = [
        "//cyber:binary",
        "@com_github_google_glog//:glog",
    ],
)

cc_test(
    name = "log_test",
    size = "small",
    srcs = ["log_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "environment",
    hdrs = ["environment.h"],
    deps = [
        "//cyber/common:log",
    ],
)

cc_test(
    name = "environment_test",
    size = "small",
    srcs = ["environment_test.cc"],
    deps = [
        ":environment",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "global_data",
    srcs = ["global_data.cc"],
    hdrs = ["global_data.h"],
    data = [
        "//cyber:cyber_conf",
    ],
    deps = [
        "//cyber/base:atomic_hash_map",
        "//cyber/base:atomic_rw_lock",
        "//cyber/common:environment",
        "//cyber/common:file",
        "//cyber/common:macros",
        "//cyber/common:util",
        "//cyber/proto:cyber_conf_cc_proto",
    ],
)

cc_test(
    name = "macros_test",
    size = "small",
    srcs = ["macros_test.cc"],
    deps = [
        "//cyber",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "macros",
    hdrs = ["macros.h"],
    deps = [
        "//cyber/base:macros",
    ],
)

cc_library(
    name = "time_conversion",
    hdrs = ["time_conversion.h"],
)

cc_library(
    name = "types",
    hdrs = ["types.h"],
)

cc_library(
    name = "util",
    hdrs = ["util.h"],
)

cpplint()
