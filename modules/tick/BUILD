load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "tick_component_lib",
    srcs = [
        "tick_component.cc",
    ],
    hdrs = [
        "tick_component.h",
    ],
    copts = ['-DMODULE_NAME=\\"tick\\"'],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/time",
        "//modules/localization/proto:localization_proto",
        "//modules/tick/proto:tick_conf_proto",
        "//modules/tick/proto:tick_proto",
    ],
)

cc_test(
    name = "tick_component_test",
    size = "small",
    srcs = ["tick_component_test.cc"],
    deps = [
        ":tick_component_lib",
        "@gtest//:main",
    ],
)

cc_binary(
    name = "libtick_component.so",
    linkshared = True,
    linkstatic = False,
    deps = [":tick_component_lib"],
)

cpplint()
