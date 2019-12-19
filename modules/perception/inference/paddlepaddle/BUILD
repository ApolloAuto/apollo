load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "paddle_net_lib",
    srcs = ["paddle_net.cc"],
    hdrs = ["paddle_net.h"],
    linkopts = [
        "-lsnappy",
        "-lz",
        "-lsnappy",
        "-lrt",
        "-ldl",
        "-lpthread",
    ],
    deps = [
        "//cyber/common:log",
        "//external:gflags",
        "//modules/perception/inference:inference_lib",
        "@boost",
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
