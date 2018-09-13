load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

# https://docs.bazel.build/versions/master/be/c-cpp.html#cc_library

cc_binary(
    name = "libwatchdog_component.so",
    deps = [":watchdog_component_lib"],
    linkopts = ["-shared"],
    linkstatic = False,
)

cc_library(
    name = "watchdog_component_lib",
    srcs = ["watchdog_component.cc",
            "watchdog.cc"],
    hdrs = ["watchdog_component.h",
            "watchdog.h"],
    deps = [
        "//framework:cybertron"
        ],
    copts = ['-DMODULE_NAME=\\"watchdog\\"']
)

cpplint()
