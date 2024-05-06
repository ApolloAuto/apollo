load("//tools:cpplint.bzl", "cpplint")
load("//tools:apollo_package.bzl", "apollo_cc_library", "apollo_package")

apollo_cc_library(
    name = "latency_recorder",
    srcs = ["latency_recorder.cc"],
    hdrs = ["latency_recorder.h"],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/latency_recorder/proto:latency_record_cc_proto",
        "//modules/common/util:util_tool",
    ],
)

apollo_package()

cpplint()
