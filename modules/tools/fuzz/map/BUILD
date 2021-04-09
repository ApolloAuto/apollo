load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "hdmap_fuzz",
    srcs = [
        "hdmap_fuzz.cc",
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
        "//modules/map/hdmap:hdmap",
        "//modules/tools/fuzz/map/proto:hdmap_fuzz_proto",
        "@libprotobuf_mutator//:mutator",
    ],
)

cpplint()