load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "lidar_ex_checker_fuzz",
    srcs = [
        "lidar_ex_checker_fuzz.cc",
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
        "//modules/calibration/lidar_ex_checker:lidar_ex_checker_lib",
        "//modules/tools/fuzz/calibration/proto:lidar_ex_checker_fuzz_proto",
        "@libprotobuf_mutator//:mutator",
    ],
)

cc_binary(
    name = "republish_msg_fuzz",
    srcs = [
        "republish_msg_fuzz.cc",
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
        "//modules/calibration/republish_msg:republish_msg_lib",
        "//modules/tools/fuzz/calibration/proto:republish_msg_fuzz_proto",
        "@libprotobuf_mutator//:mutator",
    ],
)

cpplint()