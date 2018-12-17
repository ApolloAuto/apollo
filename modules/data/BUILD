load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "data_lib",
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/util:message_util",
        "//modules/data/common:data_gflags",
        "//modules/data/proto:data_conf_proto",
        "//modules/data/proto:data_proto",
        "//modules/data/recorder:record_service",
    ],
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
    linkshared = True,
    deps = [":data_component_lib"],
)

cc_test(
    name = "data_component_test",
    size = "small",
    srcs = ["data_component_test.cc"],
    deps = [
        ":data_component_lib",
        "@gtest//:main",
    ],
)

cpplint()
