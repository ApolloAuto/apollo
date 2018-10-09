load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libgnss_component.so",
    deps = [":gnss_component_lib"],
    linkopts = ["-shared"],
    linkstatic = False,
)

cc_library(
    name = "gnss_component_lib",
    srcs = ["gnss_component.cc"],
    hdrs = ["gnss_component.h"],
    deps = [
        "//modules/drivers/gnss/stream:gnss_stream",
        "//modules/common/monitor_log",
        "//framework:cybertron",
    ],
    copts = ['-DMODULE_NAME=\\"gnss\\"']
)

cpplint()
