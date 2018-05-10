load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "core",
    deps = [
        ":routing_navigator",
    ],
)

cc_library(
    name = "routing_navigator",
    srcs = [
        "navigator.cc",
    ],
    hdrs = [
        "navigator.h",
    ],
    deps = [
        ":routing_black_list_range_generator",
        ":routing_result_generator",
        "//modules/common/adapters:adapter_manager",
        "//modules/common/proto:common_proto",
        "//modules/common/time",
        "//modules/common/util",
        "//modules/routing/common:routing_gflags",
        "//modules/routing/graph",
        "//modules/routing/proto:routing_proto",
        "//modules/routing/strategy",
    ],
)

cc_library(
    name = "routing_black_list_range_generator",
    srcs = [
        "black_list_range_generator.cc",
    ],
    hdrs = [
        "black_list_range_generator.h",
    ],
    deps = [
        "//modules/routing/graph",
    ],
)

cc_library(
    name = "routing_result_generator",
    srcs = [
        "result_generator.cc",
    ],
    hdrs = [
        "result_generator.h",
    ],
    deps = [
        "//modules/common/adapters:adapter_manager",
        "//modules/common/time",
        "//modules/common/util:map_util",
        "//modules/routing/graph",
        "//modules/routing/proto:routing_proto",
    ],
)

cpplint()
