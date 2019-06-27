load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "paddle_net_lib",
    srcs = ["paddle_net.cc"],
    hdrs = ["paddle_net.h"],
    linkopts = [
        "-lglog",
        "-lsnappy",
        "-lboost_system",
        "-lboost_filesystem",
        "-lboost_thread",
        "-lboost_program_options",
        "-lz",
        "-lsnappy",
        "-lrt",
        "-ldl",
        "-lpthread",
    ],
    deps = [
        "//external:gflags",
        "//modules/perception/inference:inference_lib",
        "@com_google_protobuf//:protobuf",
        "@cuda",
        "@eigen",
        "@mklml",
        "@paddlepaddle",
        "@snappystream",
        "@xxhash",
    ],
)

cpplint()
