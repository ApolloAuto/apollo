load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/install:install.bzl", "install")

install(
    name = "install",
    library_dest = "common/lib",
    data_dest = "common",
    runtime_dest = "common/bin",
    targets = [
        ":liblatency_recorder.so",
    ],
    visibility = ["//visibility:public"],
)


cc_binary(
    name = "liblatency_recorder.so",
    srcs = [
        "latency_recorder.cc",
        "latency_recorder.h",
    ],
    linkshared = True,
    linkstatic = True,
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder/proto:latency_record_cc_proto",
        "//modules/common/util:util_tool",
    ],
)

cc_library(
    name = "latency_recorder",
    srcs = ["liblatency_recorder.so"],
    hdrs = ["latency_recorder.h"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder/proto:latency_record_cc_proto",
        "//modules/common/util:util_tool",
    ],
    visibility = ["//visibility:public"],
)

cpplint()
