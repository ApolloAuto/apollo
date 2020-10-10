load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])
ROUTING_COPTS = ['-DMODULE_NAME=\\"routing\\"']

cc_library(
    name = "routing_lib",
    srcs = ["routing.cc"],
    hdrs = ["routing.h"],
    copts = ROUTING_COPTS,
    deps = [
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/util:point_factory",
        "//modules/map/hdmap:hdmap_util",
        "//modules/routing/core",
        "//modules/routing/proto:routing_config_cc_proto",
    ],
)

cc_library(
    name = "routing_component_lib",
    srcs = ["routing_component.cc"],
    hdrs = ["routing_component.h"],
    copts = ROUTING_COPTS,
    deps = [
        ":routing_lib",
        "//cyber/time:clock",
    ],
)

cc_binary(
    name = "librouting_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":routing_component_lib"],
)

filegroup(
    name = "routing_testdata",
    srcs = glob([
        "testdata/**",
    ]),
)

cpplint()
