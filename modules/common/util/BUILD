load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "util",
    srcs = [
        "file.cc",
        "string_tokenizer.cc",
        "util.cc",
    ],
    hdrs = [
        "file.h",
        "string_tokenizer.h",
        "util.h",
    ],
    deps = [
        "//modules/common/math",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/time",
        "//modules/perception/proto:perception_proto",
    ],
)

cc_test(
    name = "util_test",
    size = "small",
    srcs = [
        "util_test.cc",
    ],
    deps = [
        ":util",
        "@gtest//:main",
    ],
)

cc_library(
    name = "lru_cache",
    hdrs = ["lru_cache.h"],
)

cc_library(
    name = "string_util",
    hdrs = [
        "string_util.h",
    ],
    deps = [
        "@com_google_protobuf//:protobuf",
    ]
)

cc_test(
    name = "string_util_test",
    size = "small",
    srcs = [
        "string_util_test.cc",
    ],
    deps = [
        ":string_util",
        "@gtest//:main",
    ],
)

cc_library(
    name = "map_util",
    hdrs = ["map_util.h"],
    deps = [
        "@com_google_protobuf//:protobuf",
    ]
)

cc_test(
    name = "string_tokenizer_test",
    size = "small",
    srcs = [
        "string_tokenizer_test.cc",
    ],
    deps = [
        ":util",
        "@gtest//:main",
    ],
)

cc_library(
    name = "factory",
    hdrs = [
        "factory.h",
    ],
    deps = [
        "//modules/common:log",
    ],
)

cc_test(
    name = "factory_test",
    size = "small",
    srcs = [
        "factory_test.cc",
    ],
    deps = [
        ":factory",
        "//modules/common",
        "//modules/common:log",
        "@gtest//:main",
    ],
)

cc_library(
    name = "dropbox",
    hdrs = [
        "dropbox.h",
    ],
    deps = [
        "//modules/common:log",
    ],
)

cc_test(
    name = "dropbox_test",
    size = "small",
    srcs = [
        "dropbox_test.cc",
    ],
    deps = [
        ":dropbox",
        "//modules/common",
        "//modules/common:log",
        "@gtest//:main",
    ],
)

cc_test(
    name = "file_test",
    size = "small",
    srcs = [
        "file_test.cc",
    ],
    tags = [
        "external",
    ],
    deps = [
        ":util",
        "//modules/common/util/testdata:simple_proto",
        "@gtest//:main",
    ],
)

cc_test(
    name = "lru_cache_test",
    size = "small",
    srcs = [
        "lru_cache_test.cc",
    ],
    deps = [
        "//modules/common/util:lru_cache",
        "@gtest//:main",
    ],
)

cc_library(
    name = "points_downsampler",
    hdrs = [
        "points_downsampler.h",
    ],
    deps = [
        "//modules/common:log",
        "//modules/common/math:vec2d",
    ],
)

cc_test(
    name = "points_downsampler_test",
    size = "small",
    srcs = [
        "points_downsampler_test.cc",
    ],
    deps = [
        ":points_downsampler",
        "@gtest//:main",
    ],
)

cpplint()
