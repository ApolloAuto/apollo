load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "control_fuzz",
    srcs = [
        "control_fuzz.cc",
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
        "//modules/control:control_lib",
        "//modules/tools/fuzz/control/proto:control_fuzz_proto",
        "@libprotobuf_mutator//:mutator",
    ],
)

cpplint()
