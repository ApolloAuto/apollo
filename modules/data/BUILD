load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "data_lib",
    deps = [
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/data/proto:data_proto",
        "//modules/data/common:data_gflags",
        "//modules/data/proto:data_conf_proto",
        "//framework:cybertron",
        "//modules/data/recorder:record_service",
    ]
)

cc_library(
    name = "data_component_lib",
    srcs = ["data_component.cc"],
    hdrs = ["data_component.h"],
    copts = ["-DMODULE_NAME=\\\"data\\\""],
    deps = [
        ":data_lib",
    ],
)

cc_binary(
    name = "libdata_component.so",
    deps = [":data_component_lib"],
    linkshared = True,
)

cc_test(
    name = "data_component_test",
    size = "small",
    srcs = ["data_component_test.cc"],
    deps = [
        "@gtest//:main",
        ":data_component_lib",
    ],
)


cpplint()
