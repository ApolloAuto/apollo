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
    linkopts = [
        "-lboost_filesystem",
    ],
    deps = [
        ":string_util",
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
    name = "disjoint_set",
    hdrs = ["disjoint_set.h"],
)

cc_test(
    name = "disjoint_set_test",
    size = "small",
    srcs = [
        "disjoint_set_test.cc",
    ],
    deps = [
        ":disjoint_set",
        "//modules/common:log",
        "@gtest//:main",
    ],
)

cc_library(
    name = "lru_cache",
    hdrs = ["lru_cache.h"],
)

cc_library(
    name = "ctpl_stl",
    hdrs = ["ctpl_stl.h"],
)

cc_test(
    name = "ctpl_stl_test",
    size = "small",
    srcs = [
        "ctpl_stl_test.cc",
    ],
    deps = [
        "//modules/common/util:ctpl_stl",
        "@gtest//:main",
    ],
)

cc_library(
    name = "color",
    hdrs = ["color.h"],
)

cc_library(
    name = "string_util",
    srcs = [
        "string_util.cc",
    ],
    hdrs = [
        "string_util.h",
    ],
    deps = [
        "@com_google_protobuf//:protobuf",
    ],
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
    ],
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

cc_test(
    name = "file_test",
    size = "small",
    srcs = [
        "file_test.cc",
    ],
    data = [
        "//modules/perception:perception_data",
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
        "//modules/common/math:geometry",
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

cc_library(
    name = "json_util",
    srcs = [
        "json_util.cc",
    ],
    hdrs = [
        "json_util.h",
    ],
    deps = [
        "//modules/common:log",
        "//third_party/json",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "json_util_test",
    size = "small",
    srcs = [
        "json_util_test.cc",
    ],
    deps = [
        ":json_util",
        "//modules/common/proto:error_code_proto",
        "@gtest//:main",
    ],
)

cc_library(
    name = "http_client",
    srcs = ["http_client.cc"],
    hdrs = ["http_client.h"],
    deps = [
        "//modules/common:log",
        "//modules/common/status",
        "//modules/common/util:string_util",
        "//third_party/json",
        "@curlpp",
    ],
)

cpplint()
