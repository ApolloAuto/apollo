load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "util",
    srcs = [
        "string_tokenizer.cc",
        "util.cc",
    ],
    hdrs = [
        "blocking_queue.h",
        "string_tokenizer.h",
        "util.h",
    ],
    deps = [
        ":color",
        ":disjoint_set",
        ":file_util",
        ":http_client",
        ":json_util",
        ":lru_cache",
        ":message_util",
        ":points_downsampler",
        ":string_util",
        "//modules/common/math",
        "//modules/common/proto:pnc_point_proto",
        "//modules/common/time",
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
        "//modules/common/util/testdata:simple_proto",
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
        "//cyber",
        "@gtest//:main",
    ],
)

cc_library(
    name = "lru_cache",
    hdrs = ["lru_cache.h"],
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

cc_library(
    name = "message_util",
    hdrs = [
        "message_util.h",
    ],
    deps = [
        ":file_util",
        ":string_util",
        "//modules/common/time",
        "@com_google_protobuf//:protobuf",
    ],
)

cc_test(
    name = "message_util_test",
    size = "small",
    srcs = [
        "message_util_test.cc",
    ],
    tags = [
        "external",
    ],
    deps = [
        ":message_util",
        "//modules/common/util/testdata:simple_proto",
        "@gtest//:main",
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
        "//cyber",
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
        "//cyber",
        "//modules/common",
        "@gtest//:main",
    ],
)

cc_library(
    name = "file_util",
    hdrs = [
        "file.h",
    ],
    linkopts = [
        "-lboost_filesystem",
        "-lboost_system",
    ],
    deps = [
        "//cyber/common:file",
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
        "//cyber",
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
        "//cyber",
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
        "//cyber",
        "//modules/common/status",
        "//modules/common/util:string_util",
        "//third_party/json",
        "@curlpp",
    ],
)

cpplint()
