load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "navi_planning_fuzz",
    srcs = [
        "navi_planning_fuzz.cc",
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
        "//modules/planning:navi_planning_lib",
        "//modules/tools/fuzz/planning/proto:navi_planning_fuzz_proto",
        "@libprotobuf_mutator//:mutator",
    ],
)

cc_binary(
    name = "std_planning_fuzz",
    srcs = [
        "std_planning_fuzz.cc",
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
        "//modules/planning:std_planning_lib",
        "//modules/tools/fuzz/planning/proto:std_planning_fuzz_proto",
        "@libprotobuf_mutator//:mutator",
    ],
)

cpplint()