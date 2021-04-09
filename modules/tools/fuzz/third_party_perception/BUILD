load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "third_party_perception_fuzz",
    srcs = [
        "third_party_perception_fuzz.cc",
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
        "//modules/third_party_perception:third_party_perception_lib",
        "//modules/tools/fuzz/third_party_perception/proto:third_party_perception_fuzz_proto",
        "@libprotobuf_mutator//:mutator",
    ],
)

cpplint()