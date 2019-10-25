load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "latency_record_proto",
    deps = [":latency_record_proto_lib"],
)

proto_library(
    name = "latency_record_proto_lib",
    srcs = ["latency_record.proto"],
    deps = ["//modules/common/proto:header_proto_lib"],
)

cpplint()
