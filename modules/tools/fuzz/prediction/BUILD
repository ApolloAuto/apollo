load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "prediction_fuzz",
    srcs = [
        "prediction_fuzz.cc",
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
        "//modules/prediction:prediction_lib",
        "//modules/tools/fuzz/prediction/proto:prediction_fuzz_proto",
        "@libprotobuf_mutator//:mutator",
    ],
)

cpplint()