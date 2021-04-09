load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "guardian_fuzz",
    srcs = [
        "guardian_fuzz.cc",
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
        "//modules/guardian:guardian_lib",
        "//modules/tools/fuzz/guardian/proto:guardian_fuzz_proto",
        "@libprotobuf_mutator//:mutator",
    ],
)

cpplint()