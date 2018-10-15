load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "common",
    deps = [
        "//cybertron/common:environment",
        "//cybertron/common:file",
        "//cybertron/common:global_data",
        "//cybertron/common:log",
        "//cybertron/common:macros",
        "//cybertron/common:time_conversion",
        "//cybertron/common:types",
        "//cybertron/common:util",
    ],
)

cc_library(
    name = "file",
    srcs = [
          "file.cpp",
    ],
    hdrs = [
          "file.h",
    ],
    deps = [
        "//cybertron/common:log",
    ],
)


cc_test(
    name = "file_test",
    size = "small",
    srcs = [
        "file_test.cpp",
    ],
    deps = [
        "//cybertron",
        "//cybertron/proto:unit_test_cc_proto",
        "@gtest//:main",
    ],
)

cc_library(
    name = "log",
    hdrs = [
          "log.h",
    ],
    deps = [
        "//cybertron:init_lib",
    ],
)

cc_test(
    name = "logfileobject_test",
    size = "small",
    srcs = [
        "logfileobject_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_test(
    name = "log_test",
    size = "small",
    srcs = [
        "log_test.cpp",
    ],
    deps = [
        "//cybertron",
        "@gtest//:main",
    ],
)

cc_library(
    name = "environment",
    hdrs = [
          "environment.h",
    ],
    deps = [
#"//cybertron/common:log",
    ],
)

cc_library(
    name = "global_data",
    srcs = [
          "global_data.cpp",
    ],
    hdrs = [
          "global_data.h",
    ],
    deps = [
        "//cybertron/base:atomic_hash_map",
        "//cybertron/base:atomic_rw_lock",
        "//cybertron/common:environment",
        "//cybertron/common:file",
        "//cybertron/common:macros",
        "//cybertron/common:util",
        "//cybertron/proto:cyber_config_cc_proto",
    ],
)

cc_library(
    name = "macros",
    hdrs = [
          "macros.h",
    ],
)

cc_library(
    name = "time_conversion",
    hdrs = [
          "time_conversion.h",
    ],
)

cc_library(
    name = "types",
    hdrs = [
          "types.h",
    ],
)

cc_library(
    name = "util",
    hdrs = [
          "util.h",
    ],
)


cpplint()
