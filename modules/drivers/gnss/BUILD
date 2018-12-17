load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libgnss_component.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":gnss_component_lib"],
)

cc_library(
    name = "gnss_component_lib",
    srcs = ["gnss_component.cc"],
    hdrs = ["gnss_component.h"],
    copts = ['-DMODULE_NAME=\\"gnss\\"'],
    deps = [
        "//cyber",
        "//modules/common/monitor_log",
        "//modules/drivers/gnss/stream:gnss_stream",
    ],
)

cpplint()
