load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "latency_recorder",
    srcs = ["latency_recorder.cc"],
    hdrs = ["latency_recorder.h"],
    deps = [
        "//cyber",
        "//cyber/common:log",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder/proto:latency_record_proto",
        "//modules/common/util:message_util",
    ],
)

cpplint()
