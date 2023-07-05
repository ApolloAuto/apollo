load("@rules_cc//cc:defs.bzl", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
ROUTING_COPTS = ['-DMODULE_NAME=\\"routing\\"']

cc_library(
    name = "core",
    deps = [
        ":routing_navigator",
    ],
)

cc_library(
    name = "routing_navigator",
    srcs = ["navigator.cc"],
    hdrs = ["navigator.h"],
    copts = ROUTING_COPTS,
    deps = [
        ":routing_black_list_range_generator",
        ":routing_result_generator",
        "//modules/common/util",
        "//modules/routing/strategy",
    ],
)

cc_library(
    name = "routing_black_list_range_generator",
    srcs = ["black_list_range_generator.cc"],
    hdrs = ["black_list_range_generator.h"],
    copts = ROUTING_COPTS,
    deps = [
        "//modules/routing/graph",
    ],
)

cc_library(
    name = "routing_result_generator",
    srcs = ["result_generator.cc"],
    hdrs = ["result_generator.h"],
    copts = ROUTING_COPTS,
    deps = [
        "//modules/routing/graph",
    ],
)

cpplint()
