load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "routing_lib",
    srcs = ["routing.cc"],
    hdrs = ["routing.h"],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/monitor_log",
        "//modules/common/util:point_factory",
        "//modules/map/hdmap:hdmap_util",
        "//modules/routing/core",
    ],
)

cc_library(
    name = "routing_component_lib",
    srcs = ["routing_component.cc"],
    hdrs = ["routing_component.h"],
    copts = ['-DMODULE_NAME=\\"routing\\"'],
    deps = [
        ":routing_lib",
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
