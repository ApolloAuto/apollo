load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "routing_fuzz",
    srcs = [
        "routing_fuzz.cc",
    ],
    copts = [
        "-fsanitize=fuzzer,address,undefined",
        "-Iexternal/libprotobuf_mutator/src/",
    ],
    linkopts = [
        "-fsanitize=fuzzer,address,undefined",
        "-lubsan",
    ],
    deps = [
        "//modules/routing:routing_lib",
        "//modules/tools/fuzz/routing/proto:routing_fuzz_proto",
        "@libprotobuf_mutator//:mutator",
    ],
)

cpplint()