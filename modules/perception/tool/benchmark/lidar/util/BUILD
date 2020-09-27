load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "types",
    hdrs = ["types.h"],
    deps = [
        "@local_config_pcl//:pcl",
    ],
)

cc_library(
    name = "object_supplement",
    srcs = ["object_supplement.cc"],
    hdrs = ["object_supplement.h"],
    deps = [
        "@eigen",
    ],
)

cc_library(
    name = "object",
    srcs = ["object.cc"],
    hdrs = ["object.h"],
    deps = [
        ":object_supplement",
        ":types",
        "//modules/common/util:string_util",
        "@com_google_absl//absl/strings",
        "@eigen",
        "@local_config_pcl//:pcl",
    ],
)

cc_library(
    name = "geo_util",
    srcs = ["geo_util.cc"],
    hdrs = ["geo_util.h"],
    deps = [
        ":types",
        "@eigen",
    ],
)

cc_library(
    name = "io_util",
    srcs = ["io_util.cc"],
    hdrs = ["io_util.h"],
    deps = [
        ":geo_util",
        ":object",
        "@local_config_pcl//:pcl",
    ],
)

cc_library(
    name = "visibility",
    srcs = ["visibility.cc"],
    hdrs = ["visibility.h"],
    deps = [
        ":geo_util",
        ":object",
    ],
)

cc_library(
    name = "option_parser",
    srcs = ["option_parser.cc"],
    hdrs = ["option_parser.h"],
    deps = [
        "//modules/common/util:string_util",
        "@com_google_absl//absl/strings",
    ],
)

cc_library(
    name = "benchmark_util",
    deps = [
        ":geo_util",
        ":io_util",
        ":object",
        ":object_supplement",
        ":option_parser",
        ":visibility",
    ],
)

cpplint()
