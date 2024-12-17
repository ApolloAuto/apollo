load("//tools:cpplint.bzl", "cpplint")
load("//tools/proto:proto.bzl", "proto_library", "apollo_py_binary")
load("//tools:apollo_package.bzl", "apollo_package", "apollo_cc_binary")

package(default_visibility = ["//visibility:public"])

apollo_py_binary(
    name = "cyber_benchmark",
    srcs = ["cyber_benchmark.py"],
)

apollo_cc_binary(
    name = "cyber_benchmark_reader",
    srcs = [
        "cyber_benchmark_reader.cc",
    ],
    linkopts = [
        "-pthread",
        "-lprofiler",
        "-ltcmalloc",
    ],
    deps = [
        "//cyber",
        ":benchmark_msg_proto",
    ],
)

apollo_cc_binary(
    name = "cyber_benchmark_writer",
    srcs = [
        "cyber_benchmark_writer.cc",
    ],
    linkopts = [
        "-pthread",
        "-lprofiler",
        "-ltcmalloc",
    ],
    deps = [
        "//cyber",
        "//cyber/time:cyber_time",
        ":benchmark_msg_proto",
    ],
)

proto_library(
    name = "benchmark_msg_proto",
    srcs = ["benchmark_msg.proto"],
)

apollo_package()
cpplint()