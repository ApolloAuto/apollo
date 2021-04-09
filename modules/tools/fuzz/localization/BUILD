load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "rtk_localization_fuzz",
    srcs = [
        "rtk_localization_fuzz.cc",
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
        "//modules/localization/rtk:rtk_localization",
        "//modules/tools/fuzz/localization/proto:rtk_localization_fuzz_proto",
        "@libprotobuf_mutator//:mutator",
    ],
)

cc_binary(
    name = "msf_localization_fuzz",
    srcs = [
        "msf_localization_fuzz.cc",
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
        "//modules/localization/msf:msf_localization",
        "//modules/tools/fuzz/localization/proto:msf_localization_fuzz_proto",
        "@libprotobuf_mutator//:mutator",
    ],
)

cpplint()