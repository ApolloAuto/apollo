load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "dummy_plugin",
    srcs = ["dummy_plugin.cc"],
    hdrs = ["dummy_plugin.h"],
    deps = [
        "//modules/monitor/sysmon/base:sysmon_base",
    ],
)

cc_binary(
    name = "plugin_loader",
    srcs = ["plugin_loader.cc"],
    deps = [
        ":dummy_plugin",
    ],
    linkopts= [ "-ldl" ],
)

cpplint()
